
:- module(knowrob_action_parser, [
     parser_create/1,
     parser_create/2,
     detect_activity/3,
     detect_activity2/3
]).
/** <module> Manipulation Activity Parser.

We exploit our action model to detect activities in streams
of observed processes. Observed processes are force dynam-
ics, states, and motions that are used to define actions in
our model. We use temporal constraints in action definitions
to build an endpoint sequence graph (ESG) of constituent
endpoints that represents their temporal ordering. Activity
detection is implemented through a top-down depth-first
parser that uses ESGs as grammar. An activity is detected
whenever the parser is able to reach an ESG endnode
following only nodes that can be unified with the stream
of observed event endpoints which are used as tokens by the
parser.

The parser, named Manipulation Activity Parser (MAP),
builds a parse tree of detected action terms. We say that MAP
parse trees are interpretations of observed event endpoints.
Multiple interpretations may be provided by the parser. In-
terpretations are scored according to the number of observed
endpoints they explain. We select the interpretation that
explains the maximum number of endpoints. MAP parse
trees also imply temporal constraints on action constituents
such as that a lifting motion starts when a grasped artifact
looses contact to its supporting surface. The motion segmen-
tation represented in MAP parse trees is only partial because
endpoints of motions are often not observable nor do they co-
occur with force dynamics endpoints which are observable
by an external viewer.

TODO detect "silence" in long token streams
@author Daniel Beßler
*/

:- use_module(library('debug')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/temporal')). % `interval/2`

:- debug(activity_parser).

:- rdf_meta is_typed_endpoint(t,r),
            parser_grammar(?,r,r,t),
            parser_create_grammar(+,r),
            parser_create(-,t),
            detect_activity(+,t,t),
            detect_activity2(+,t,t).

:- dynamic parser_grammar/4. % Parser, Workflow, Task Concept, Sequence Graph

random_id(Id) :-
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Id, Alpha_random).
           
% generate unique parser id
parser_unique_id(Parser) :-
  random_id(P),
  ( parser_grammar(P,_,_,_) ->
    parser_unique_id(Parser) ;
    Parser = P ).

no_grammar(Parser) :-
    print_message(warning, no_grammar(Parser)),
    fail.
no_grammar(Parser,Tsk) :-
    print_message(warning, no_grammar(Parser,Tsk)),
    fail.

parser_grammar_(Parser,WF,Tsk,GraphChild) :-
  var(Tsk), !,
  ( parser_grammar(Parser,WF,Tsk,GraphChild) *->
    true ; no_grammar(Parser) ).
parser_grammar_(Parser,WF,Tsk,GraphChild) :-
  (( parser_grammar(Parser,WF,X,GraphChild),
     once(rdfs_subclass_of(Tsk,X)) ) *->
     true ; no_grammar(Parser,Tsk) ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Endpoint sequence graph stuff

is_typed_endpoint(-(Iri),Type) :- rdfs_subclass_of(Iri,Type),!.
is_typed_endpoint(+(Iri),Type) :- rdfs_subclass_of(Iri,Type),!.
is_typed_endpoint(Iri,Type)    :- atom(Iri), rdfs_subclass_of(Iri,Type),!.

is_action_endpoint(E)  :- is_typed_endpoint(E,dul:'Task'),!.
is_state_endpoint(E)   :- is_typed_endpoint(E,ease_state:'Gestallt'),!.
is_process_endpoint(E) :- is_typed_endpoint(E,ease_proc:'ProcessType'),!.

%% parser_create(-Parser).
%% parser_create(-Parser,+RDFGraph).
%
% Creates a new activity detection parser.
% For this, sequence graphs are created for each
% action concept asserted in the RDF triple store
% which is a subclass of actions:'PhysicalAction'.
% It's possible to limit to concepts stored in a named
% RDF graph which defaults to the 'user' graph.
%
% @param Parser Parser id.
% @param RDFGraph RDF graph name (defaults to 'user').
%
parser_create(Parser) :-
  parser_create(Parser,[]).

parser_create(Parser,Workflows) :-
  parser_unique_id(Parser),
  forall(member(WF,Workflows),(
    parser_create_grammar(Parser,WF);
    print_message(warning, invalid_grammar(WF))
  )).

parser_create_grammar(Parser,WF) :-
  % find constituents and their relation to each other
  plan_defines_task(WF,Tsk),
  ( rdfs_individual_of(Tsk,owl:'Class') ->
    TskType = Tsk ;
    kb_type_of(Tsk,TskType)
  ),
  workflow_constituents(WF,Constituents,Constraints),
  % compute the sequence graph
  print_message(informational, loading_grammar(TskType)),
  esg_truncated(Tsk,Constituents,Constraints,[Sequence,
                           PreConditions, PostConditions]),
  print_message(informational, grammar(TskType,Sequence)),
  % assert to Prolog KB
  assertz(parser_grammar(Parser,WF,TskType,[Sequence,PreConditions,PostConditions])).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

peak_token(tok(0,a,-(a),[]),[],[]) :- !.
peak_token(X,[X|Xs],[X|Xs]) :- !.

% custom parsing operator
:- op( 1200, xfx, [ :-> ]).

term_expansion(Head_0 :-> Body_0, Head_1 --> Body_n) :-
  Head_0=..[Functor|Args],
  Head_1=..[Functor,P,G,S,A|Args],
  expand_esg(P,G,S,A, Body_0, Body_n).

expand_esg(P,G_0->G_n,S_0->S_n,A_0->A_n,(X1,X2),Expanded) :-
  !,
  expand_esg_term(P,G_0->G_1,S_0->S_1,A_0->A_1,X1,Y1),
  expand_esg(P,G_1->G_n,S_1->S_n,A_1->A_n,X2,Y2),
  ( Y1 = (Z1,Z2) ->
    Expanded=(Z1,(Z2,Y2)) ;
    Expanded=(Y1,Y2) ).
expand_esg(P,G,S,A,X1,Y1) :-
  expand_esg_term(P,G,S,A,X1,Y1).
  
expand_esg_term(_P,G_0->G_1,S_0->S_0,A_0->A_0,
  {'esg'(X->Y)},
  {G_0=X,G_1=Y}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_0,
  {'peak'(Endpoint)},
  {esg_peak(G_0,E), concept_endpoint_(E,Endpoint)}) :- !.

% FIXME: ugly peak1/peak2
expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_0,
  {'peak2'(Endpoint)},
  {esg_peak(G_0,Endpoint)}) :- !.

expand_esg_term(_P,G_0->G_1,S_0->S_0,A_0->A_0,
  {'pop'(Endpoint)},
  {esg_pop(G_0,E,G_1), concept_endpoint_(E,Endpoint)}) :- !.

% FIXME: causes a warning?!?
expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_0,
  {'peak_token'(Token)},
  peak_token(Token)) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_1,
  {'preceded_by'(WF,Pre,Confidence)},
  {activity_preceded_by(WF,Pre,S_0,A_0->A_1,Confidence)}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,Actors,
  {'proceed_with'(WF,Endpoint,Participants)},
  {proceed_with_participants(WF,Endpoint,Actors,Participants)}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,Actors,
  {'proceed_without'(WF,Endpoint,Participants)},
  {proceed_without_participants(WF,Endpoint,Actors,Participants)}) :- !.
  

expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_0,{X},{X}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_1,A_0->A_0,
               [Token],
               ([X],{X=Token,update_states(S_0->S_1,X)})) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_0,[],[]) :- !.

expand_esg_term(P,G,S,A,Term,Expanded) :-
  Term=..[Functor|Args],
  Expanded=..[Functor,P,G,S,A|Args].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% entity states

update_states(States_0->States_1,tok(_Time,_,-(Event),Participants)) :- !,
  kb_type_of(Event,EvtType),
  set_entity_states(States_0->States_1,Participants,[EvtType,Event,Participants]).
update_states(States_0->States_1,tok(_Time,_,+(Event),Participants)) :- !,
  kb_type_of(Event,EvtType),
  unset_entity_states(States_0->States_1,Participants,[EvtType,Event,Participants]).

unset_state(States_0->States_1,Key,Value) :-
  del_dict(Key,States_0,Value,States_1).
set_state(States_0->States_1,Key,Value) :-
  put_dict(Key,States_0,Value,States_1).
get_state(States,Key,Value) :-
  get_dict(Key,States,Value).

get_or_create_state(States,Key,Default,Value) :-
  once(
    get_state(States,Key,Value) ;
    Value = Default ).


unset_entity_states(S_0->S_0,[],_) :- !.
unset_entity_states(S_0->S_n,[X|Xs],Val) :-
  unset_entity_state(S_0->S_1,X,Val),
  unset_entity_states(S_1->S_n,Xs,Val).

unset_entity_state(States_0->States_0,E,[EvtType,Event,Participants]) :-
  \+ (
    entity_state(States_0,E,EvtType,[-(Event),PE]),
    subset(Participants,PE)
  ), !.
unset_entity_state(States_0->States_1,E,[EvtType,Event,Participants]) :-
  get_state(States_0,E,ES_0),
  get_state(ES_0,EvtType,Events_0),
  delete(Events_0,[-(Event),_],Events_1),
  set_state(ES_0->ES_1,EvtType,[[+(Event),Participants]|Events_1]),
  set_state(States_0->States_1,E,ES_1), !.


set_entity_states(S_0->S_0,[],_) :- !.
set_entity_states(S_0->S_n,[X|Xs],Val) :-
  set_entity_state(S_0->S_1,X,Val),
  set_entity_states(S_1->S_n,Xs,Val).

set_entity_state(S_0->S_0, E, [EvtType,Event,Participants]) :-
  entity_state(S_0,E,EvtType,[-(Event),PE]),
  subset(Participants,PE), !.
set_entity_state(S_0->S_1, E,[EvtType,Event,Participants]) :-
  get_or_create_state(S_0,E,states{},ES_0),
  get_or_create_state(ES_0,EvtType,[],Events_0),
  ( member([Event,_],Events_0) ->
    S_1 = S_0 ; (
    delete_previous_events(Events_0->Events_1,Participants),
    set_state(ES_0->ES_1,EvtType,[[-(Event),Participants]|Events_1]),
    set_state(S_0->S_1,E,ES_1)
  )).

entity_state(States_0,E,EvtType,[Event,Participants]) :-
  get_state(States_0,E,ES),
  get_state(ES,EvtType,Events),
  member([Event,Participants],Events),!.
  
delete_previous_events([]->[],_) :- !.
delete_previous_events([[X,P_X]|Xs]->Events_n,P_0) :-
  % delete events with identical participants!
  delete_previous_events(Xs->Events_1,P_0),
  ( subset(P_X,P_0) ->
    Events_n=Events_1 ;
    Events_n=[[X,P_X]|Events_1] ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% activity actors

proceed_with_participants(WF,Endpoint,A0->A1,Participants) :-
  % find all roles of the concept associated to an endpoint
  endpoint_type(Endpoint,Concept),
  concept_roles_(Concept,C_Roles),
  % create a binding and try to apply it in current context
  binding_create_(Participants,C_Roles,Binding),
  proceed_with_binding(WF,A0->A1,Binding).

proceed_without_participants(_WF,_Endpoint,A_0->A_0,Participants) :-
  % allow skipping endpoint in case none of the
  % participants is bound to the activity context.
  forall(
    member(O,Participants),
    \+ member([O,_],A_0)
  ).

proceed_with_binding(_WF,[]->A_new,A_new) :- !.
proceed_with_binding(_WF,A_0->A_1,A_new) :-
  apply_role_assignments_(A_new,A_0->A_1).


apply_role_assignments_([],A_0->A_0) :- !.
apply_role_assignments_([First|Rest],A_0->A_X) :-
  apply_role_assignment_(First,A_0->A_1),
  apply_role_assignments_(Rest,A_1->A_X).

apply_role_assignment_([O0,O0_R],A_0->_) :-
  % avoid conflicting assignment
  member(R_dup,O0_R),
  member([O1,O1_R],A_0),
  O0 \= O1,
  member(R_dup,O1_R),!,
  fail.
apply_role_assignment_([O,O_R0],A_0->A_1) :-
  member([O,O_R1],A_0),!,
  append(O_R0,O_R1,O_R),
  list_to_set(O_R,O_R_Set),
  binding_update_([O,O_R_Set],A_0->A_1).
apply_role_assignment_(NewBinding,
  A_0->[NewBinding|A_0]).


binding_create_(Objects,Roles,Bindings) :-
  % find object-role pairs
  binding_create__(Objects,Roles,ObjectRole),
  % merge pairs to object-list(role) pairs
  findall([O,OR], (
    member(O,Objects),
    findall(OR0, member([O,OR0],ObjectRole), OR),
    OR \= []
  ), Bindings).
binding_create__(_,[],[]) :- !.
binding_create__(Objects,[R|Rs],[[O,R]|Rest]) :-
  member(O,Objects),
  once((
    property_range(R, dul:classifies, O_Type),
    kb_type_of(O, O_Type)
  )),
  binding_create__(Objects,Rs,Rest).

binding_update_(_,[]->[]) :- !.
binding_update_([O,OR],[[O,_]|Xs]->[[O,OR]|Ys]) :-
  !, binding_update_([O,OR],Xs->Ys).
binding_update_(New,[X|Xs]->[X|Ys]) :-
  binding_update_(New,Xs->Ys).

apply_role_bindings_(_Plan, _Tsk, _Bindings, []->[]) :- !.
apply_role_bindings_(Plan, Tsk, Bindings, [B0|Rest0]->[B1|Rest1]) :-
  apply_role_binding_(Plan, Tsk, Bindings, B0->B1),
  apply_role_bindings_(Plan, Tsk, Bindings, Rest0->Rest1).

apply_role_binding_(Plan, _Tsk,
    Bindings, [Obj,Roles0]->[Obj,Roles1]) :-
  findall(Role, (
    member(Role,Roles0) ;
    ( rdf_has(Plan, ease_wf:hasBinding, Binding),
      rdf_has(Binding, ease_wf:hasBindingRole, X0),
      rdf_has(Binding, ease_wf:hasBindingFiller, X1),
      ( member(X0,Roles0) -> Role = X1 ;
      ( member(X1,Roles0) -> Role = X0 ; fail )),
      % avoid duplicates
      \+ member(Role,Roles0),
      % avoid conflicting assignment
      \+ (
        member([_,Roles2], Bindings),
        member(Role,Roles2)
      )
    )
  ), Roles1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

activity_preceded_by(WF,ESG,S,A,C) :-
  esg_endpoints(ESG,Endpoints),
  activity_preceded_by_(WF,Endpoints,S,A,C).

activity_preceded_by_(_,[],_,A0->A0,1.0) :- !.
activity_preceded_by_(WF,Endpoints,S,A0->A1,C) :-
  % check how many endpoints were observed that would sattisfy
  % the pre-conditionsz
  findall([Participants,C_Roles], (
    member(Endpoint,Endpoints),
    endpoint_type(Endpoint,Concept),
    %%
    kb_type_of(Concept,ConceptType),
    entity_state(S,_,ConceptType, [Evt,Participants]),
    endpoint_polarization(Evt,Endpoint),
    %%
    concept_roles_(Concept,C_Roles)
  ), ParticipantsAndRoles),
  %% confidence
  length(Endpoints,Total),
  length(ParticipantsAndRoles,Actual),
  C is Actual / Total,
  %%%
  binding_create2_(ParticipantsAndRoles,Bindings_pre),
  proceed_with_binding(WF,A0->A1,Bindings_pre).

binding_create2_([],[]) :- !.
binding_create2_([[P,R]|Rest],[B|B_Rest]) :-
  binding_create_(P,R,B),
  binding_create2_(Rest,B_Rest).
  
endpoint_polarization(-(_),-(_)).
endpoint_polarization(+(_),+(_)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parse a single typed phase endpoint.
phase_endpoint(WF,Endpoint,[phase(Endpoint,Event,Participants)]) :->
  % next token has matching endpoint and context
  [ tok(_Time,Event,Endpoint,Participants) ],
  { 'peak2'(Endpoint2) },
  { 'proceed_with'(WF,Endpoint2,Participants) }.

phase_endpoint(WF,Endpoint,ParseTree) :->
  % skip tokens of some other activity context
  [ tok(_Time,_Event,Endpoint1,Participants) ],
  { 'proceed_without'(WF,Endpoint1,Participants) },
  phase_endpoint(WF,Endpoint,ParseTree).

%phase_endpoint(_WF,Endpoint,[skipped]) :->
  %% skip ESG endpoints, i.e., endpoints which were not observed.
  %% - only allow skipping if next token is not matching endpoint
  %% TODO: allow to disable skipping by a flag
  %{ 'peak_token'(tok(_,_,E1,_Participants)) },
  %{ concept_endpoint_(E1,E2) },
  %{ E2 \= Endpoint }.

constituent(WF,-(Tsk),[action(SubWF,Tsk,Term,Confidence)]) :->
  { is_action_endpoint(Tsk) },
  sub_activity(WF, action(SubWF,Tsk,Term,Confidence)).

constituent(WF,Endpoint,PhaseTerm) :->
  { is_state_endpoint(Endpoint) ;
    is_process_endpoint(Endpoint) },
  phase_endpoint(WF, Endpoint, PhaseTerm),
  { 'pop'(Endpoint) }.

% parse endpoints of action constituents
constituents(_,[])        :-> { 'esg'([]->[]) }.  % stop if graph is empty
constituents(_WF, Tsk,[]) :-> { 'peak'(+(Tsk)) }. % stop at +(Act0) endpoint
constituents(WF, Tsk,[X|Xs]) :->
  { 'peak'(Endpoint) },
  { \+ endpoint_type(Endpoint,Tsk) },
  constituent(WF,Endpoint,[X]),
  constituents(WF,Tsk,Xs).


% parse {PreConditions}[-(Tsk),...,+(Tsk)]
activity(action(WF,Tsk,Constituents,Confidence),[Pre,_Post]) :->
  { 'pop'(-(Tsk)) },
  { 'preceded_by'(WF,Pre,PreConfidence) },
  constituents(WF,Tsk,X),
  { delete(X,skipped,Constituents) },
  { Constituents \= [] },
  { activity_confidence(WF,Constituents,PreConfidence,Confidence) },
  % TODO: can we handle post conditions? e.g. surface contact after dropping.
  %       these are ignored at the moment. maybe these could be used to imply tokens which were not observed.
  %{ 'post_conditions'(Tsk,_Post) },
  { 'pop'(+(Tsk)) }.


sub_activity(Parser,ESG_0->ESG_n,States,A_0->A_n,
             Parent_WF,action(WF,Tsk,Term,Confidence)) -->
  { parser_grammar_(Parser,WF,Tsk,[ESG_1|TskConditions]) },
  { rdf_has(WF,ease:isPlanFor,Tsk_1) },
  { esg_join(ESG_0,[Tsk_1,ESG_1],ESG_2) },
  % assign roles of WF/ACT given bindings from the parent workflow
  { apply_role_bindings_(Parent_WF, Tsk, A_0, A_0->A_1) },
  activity(Parser,ESG_2->ESG_n,States,A_1->A_x,
           action(WF,Tsk,Term,Confidence),
           TskConditions),
  % assign roles of parent TSK/WF given role assignments
  % inferred in activity predicate, and bindings from WF.
  % this will fail in case there are conflicting instantiations!
  { apply_role_bindings_(WF, Tsk, A_x, A_x->A_n) }.

% parse [*,-(Act),...,+(Act),*]
% TODO idea: re-start parser after each composite action
%        - implement "parsing" states, or prepend state tokens
%        - then yield only actions up to some token, which are complete then?
%        - could be hard to recoverparser state
%           - tokens,graph,states,actors
%           - diffeent graphs used, at different stages,and different actors
%           - also at different stages in token sequence
%        - can re-parsing allprevious tokens be avoided?
some_activity(Parser, Tsk, States, action(WF,Tsk,ActTerm,Confidence)) -->
  { parser_grammar_(Parser,WF,Tsk,[Graph|ActConditions]) },
  activity(Parser,Graph->[],States,[]->_,
           action(WF,Tsk,ActTerm,Confidence),
           ActConditions).
some_activity(Parser, Tsk, States_0->States_n, ActTerm) -->
  [T], % skip token
  { update_states(States_0->States_1,T) },
  some_activity(Parser, Tsk, States_1->States_n, ActTerm).

%% detect_activity(+Parser,+Tokens,-Interpretation).
%
% Detects activities in given token sequence.
%
% @param Parser Parser id.
% @param Tokens Token sequence.
% @param Interpretation An interpretation of the activity represented by the tokens.
%
%detect_activity(Parser,Tokens,Output) :-
  %dsg_new(DSG),
  %forall(
    %phrase(some_activity(Parser,_,states{}->_,ActionTerm), Tokens, _),
    %dsg_push(DSG,ActionTerm)),
  %dsg_count(DSG,Count),
  %debug(activity_parser,
    %'the number of detected actions is ~d', [Count]),
  %dsg_best(DSG,BestDetection),
  %dsg_free(DSG).

% TODO to be removed
detect_activity(Parser,Tokens,Output) :-
  findall(ActionTerm,
    phrase(some_activity(Parser,_,states{}->_,ActionTerm), Tokens, _),
    Unfiltered),
  filter_detections(Unfiltered,[]->Output).

% TODO to be removed
detect_activity2(Parser,Tokens,Interpretation) :-
  detect_activity(Parser,Tokens,ActionTerms),
  findall([L0,L1,X], (
    phrase(interpretation([]->E,[]->X),ActionTerms,[]),
    length(E,L0_), L0 is -L0_,
    length(X,L1)
  ), Xs),
  sort(Xs, [[_,_,Interpretation]|_]).

% TODO to be removed
detect_activity2(Parser,Tokens,Interpretation) :-
  detect_activity(Parser,Tokens,ActionTerms), !,
  assertz(best_interpretation(Parser,0,1,0,x)),
  forall(phrase(interpretation([]->E,[]->X),ActionTerms,[]), (
    length(E,L0),
    length(X,L1),
    accumulate_confidence(X,L2),
    once(best_interpretation(Parser,Best0,Best1,Best2,_)),
    (( L0 > Best0 ;
     ( L0 is Best0 , L1 < Best1 ) ;
     ( L0 is Best0 , L1 is Best1, L2 < Best2 )) -> (
      %retractall(best_interpretation(Parser,_,_,_)),
      asserta(best_interpretation(Parser,L0,L1,L2,X)) );
      true )
  )),
  once(best_interpretation(Parser,_,_,_,Interpretation)),
  Interpretation \= x,
  retractall(best_interpretation(Parser,_,_,_,_)).

% TODO to be removed
interpretation(E_0->E_n,S_0->S_n) -->
  [Term],
  { combinable(E_0,Term) },
  { term_endpoints(E_0->E_1,Term) },
  interpretation(E_1->E_n,[Term|S_0]->S_n).
interpretation(E_0->E_n,S_0->S_n) -->
  [_],
  interpretation(E_0->E_n,S_0->S_n).
interpretation(E_0->E_0,S_0->S_0) --> [].

term_endpoints(E_0->E_1,Term) :-
  findall(E,term_endpoint(Term,E),Es),
  append(E_0,Es,E_1).

term_endpoint(phase(-(_),Event,_),-(Event)) :- !.
term_endpoint(phase(+(_),Event,_),+(Event)) :- !.
term_endpoint(action(_,_,Xs,_),E) :-
  member(X,Xs),
  term_endpoint(X,E).

combinable(Es,T_1) :-
  \+ (
    term_endpoint(T_1,E),
    member(E,Es)
  ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 
% TODO: it seems a good idea to build a tree where the nodes on each level are
%       temporally distinct regions.
%       the node itself is either an action term or a
%       subtree with distinct regions contained in the parent graph.
%       the temporally distinct regions would provide 'cut' points during
%       parsing, e.g. for storing events to disk and clear them from memory.
%       duplications or specializations of terms should be detected at the leaf nodes
%       to replace/skip etc.

% delete interpretations which are subsumed by others.
filter_detections([],X0->X0) :- !.
filter_detections([T|Rest],X0->Xn) :-
  ( detection_contains(X0,T) ;
    detection_contains(Rest,T) ), !,
  filter_detections(Rest,X0->Xn).
filter_detections([T|Rest],X0->Xn) :-
  filter_detections(Rest,[T|X0]->Xn), !.

detection_contains([],_) :- fail, !.
detection_contains([X|_], X) :- !.
detection_contains([action(_,Tsk,Xs,_)|_], action(_,Tsk,Ys,_)) :-
  forall(member(Y,Ys), detection_contains(Xs,Y)), !.
detection_contains([action(_,_,Xs,_)|_],Inner) :-
  detection_contains(Xs,Inner), !.
detection_contains([_|Rest],Inner) :-
  detection_contains(Rest,Inner), !.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% confidence

activity_confidence(WF,DetectedConstituents,PreConfidence,Confidence) :-
  workflow_constituents(WF,AllConstituents,_),
  length(AllConstituents,Max), Max>0,
  findall(Event-Weighted, (
    member(Detected,DetectedConstituents),
    activity_confidence(Detected,Event,DetectedConfidence),
    Weighted is DetectedConfidence / Max),
    Xs),
  findall(X, member(X-_,Xs), Es),
  list_to_set(Es,ObservedEvents),
  findall(C_E, (
    member(E,ObservedEvents),
    findall(C, member(E-C,Xs), Cs),
    sumlist(Cs,Cs_sum),
    length(Cs,Cs_size),
    C_E is Cs_sum / Cs_size),
    Confidences),
  sumlist(Confidences,ConstituentConfidence),
  %%
  % TODO maybe better weight according to number of endpoints
  Confidence is 0.2 * PreConfidence + 0.8 * ConstituentConfidence.

activity_confidence(phase(_,Event,_),Event,1.0) :- !.
activity_confidence(action(_,Tsk,_,Confidence),Tsk,Confidence) :- !.

accumulate_confidence([], 1) :- !.
accumulate_confidence(Xs, Confidence) :-
  findall(C, (
    member(X,Xs),
    activity_confidence(X,_,C)),
    Cs),
  length(Cs,Cs_length),
  sumlist(Cs,Cs_sum),
  Confidence is Cs_sum / Cs_length.


concept_roles_(Concept,C_Roles_Set) :-
  findall(CR, (
    kb_triple(Concept,dul:isRelatedToConcept,CR),
    kb_type_of(CR,dul:'Role')
  ),C_Roles),
  list_to_set(C_Roles,C_Roles_Set).

concept_endpoint_(-(X),-(Y)) :- !, kb_type_of(X,Y).
concept_endpoint_(+(X),+(Y)) :- !, kb_type_of(X,Y).
