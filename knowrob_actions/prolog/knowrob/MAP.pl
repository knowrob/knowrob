/*  MAP.pl

    Author:        Daniel Beßler
    E-mail:        danielb@informatik.uni-bremen.de
    WWW:           http://www.ease-crc.org
    Copyright (C): 2018, University of Bremen

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    As a special exception, if you link this library with other files,
    compiled with a Free Software compiler, to produce an executable, this
    library does not by itself cause the resulting executable to be covered
    by the GNU General Public License. This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

:- module('MAP', [
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

TODO detect non terminal concepts
TODO constituents with same type
TODO optional constituents
TODO repeatable constituents
TODO detect "silence" in long token streams
@author Daniel Beßler
@license GPL
*/

:- use_module(library('debug')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/flanagan')).
:- use_module(library('knowrob/temporal')). % `interval/2`

:- rdf_db:rdf_register_ns(actions, 'http://www.ease-crc.org/ont/EASE.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(motions, 'http://www.ease-crc.org/ont/EASE.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(force_dynamics, 'http://www.ease-crc.org/ont/EASE.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease, 'http://www.ease-crc.org/ont/EASE.owl#', [keep(true)]).

:- debug(activity_parser).

:- rdf_meta is_typed_endpoint(t,r),
            parser_grammar(?,r,t),
            parser_create_grammar(+,r),
            detect_activity(+,t,t),
            detect_activity2(+,t,t),
            tokenize(r,t).

:- dynamic parser_grammar/3. % Parser, Action, Sequence Graph

% FIXME: not sure why this is needed. write should display simplified
%        terms anyway. But it doesn't (always). What's the reason?
write_concept(Concept) :-
  rdf_has_prolog(Concept,rdfs:label,Label),!,
  write(''''), write(Label), write('''').
write_concept(Concept) :-
  rdf_split_url(_,Name,Concept),!,
  write(''''), write(Name), write('''').

random_id(Id) :-
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Id, Alpha_random).
           
% generate unique parser id
parser_unique_id(Parser) :-
  random_id(P),
  ( parser_grammar(P,_,_) ->
    parser_unique_id(Parser) ;
    Parser = P ).

no_grammar(Parser) :-
    write('[parser.pl] '),
    write('ERROR: no grammars loaded for parser '),
    write(Parser), nl,
    fail.
no_grammar(Parser,Act) :-
    write('[parser.pl] '),
    write('ERROR: no '),
    write_concept(Act),
    write(' grammar loaded for parser '),
    write(Parser), nl,
    fail.

parser_grammar_(Parser,TypeIri,GraphChild) :-
  var(TypeIri), !,
  ( parser_grammar(Parser,TypeIri,GraphChild) *->
    true ; no_grammar(Parser) ).
parser_grammar_(Parser,TypeIri,GraphChild) :-
  (( parser_grammar(Parser,X,GraphChild),
     once(rdfs_subclass_of(TypeIri,X)) ) *->
     true ; no_grammar(Parser,TypeIri) ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tokenization

% TODO: remove
interval_(Event, [Begin,End]) :- interval(Event,[Begin,End]),!.
interval_(Event, [Begin,End]) :-
  % KnowRob's `interval/2` does not work with dul:'TimeInterval' dul yet
  rdf_has(Event,dul:hasTimeInterval,Interval),
  rdf_has_prolog(Interval, allen:hasIntervalBegin, Begin),
  rdf_has_prolog(Interval, allen:hasIntervalEnd, End).

%% tokenize(+Episode,?Tokens).
%
% Tokenize given episode.
% The structure of the episode symbol is that it links
% all events via knowrob:subAction.
% These are collected and mapped to a token representation.
%
% @param Episode VR episode symbol.
% @param Tokens Tokenized episode.
%
tokenize(Episode,Tokens) :-
  findall(Event, rdf_has(Episode,dul:hasConstituent,Event), Events),
  tokenize_(Events,Unsorted),
  filter_tokens(Unsorted,Filtered),
  % first argument of each token is time such that we can use Prolog builtin `sort`
  sort(Filtered,Tokens).

filter_tokens([],[]) :- !.
filter_tokens([Tok|Rest],[Tok|RestFiltered]) :-
  filter_tokens(Rest,RestFiltered).

tokenize_([], []).
tokenize_([Evt|Rest],[Tok1,Tok2|RestTokens]) :-
  tokenize_event(Evt,Tok1,Tok2),
  tokenize_(Rest,RestTokens).

tokenize_event(Event,
    tok(Begin,Event,-(EvtType),Participants),
    tok(End,  Event,+(EvtType),Participants)) :-
  rdfs_individual_of(Event,dul:'Event'), !,
  % Event types are disjoint
  % TODO: is this really safe to assume? e.g., artifact contact and effector contact.
  %       this needs special handling because different constituents may refer to
  %       different event types.
  %       Maybe would be better to stick to event individual in endpoints?
  once((
    rdf(Event,rdf:type,EvtType),
    rdfs_subclass_of(EvtType,dul:'Event')
  )),
  findall(P, rdf_has(Event, dul:hasParticipant, P), Participants),
  interval_(Event, [Begin,End]).

tokenize_event(Event,_,_) :-
  % should not happen, everything should be of type 'Event'.
  % If not this would indicate some unknown event type in the log which is
  % not mapped to flanagan ontology.
  throw(error(type_error(dul:'Event',Event), _)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Endpoint sequence graph stuff

is_typed_endpoint(-(Iri),Type) :- rdfs_subclass_of(Iri,Type),!.
is_typed_endpoint(+(Iri),Type) :- rdfs_subclass_of(Iri,Type),!.
is_typed_endpoint(Iri,Type)    :- atom(Iri), rdfs_subclass_of(Iri,Type),!.

is_action_endpoint(E)  :- is_typed_endpoint(E,dul:'Action'),!.
is_state_endpoint(E)   :- is_typed_endpoint(E,ease:'State'),!.
is_process_endpoint(E) :- is_typed_endpoint(E,ease:'PhysicsProcess'),!.

is_prehensile_motion_endpoint(E) :-
  is_typed_endpoint(E,motions:'PrehensileMotion'),!.
is_composite_action_endpoint(E) :-
  is_typed_endpoint(E,actions:'ComplexPhysicalAction'),!.

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
  parser_create(Parser,user).

parser_create(Parser,RDFGraph) :-
  parser_unique_id(Parser),
  physical_actions(Actions,RDFGraph),
  forall(member(Act,Actions),(
    parser_create_grammar(Parser,Act);(
    write('      [parser.pl] '),
    write('ERROR: Failed to build sequence graph for '), write_concept(Act), nl
  ))).

parser_create_grammar(Parser,Act) :-
  % find constituents and their relation to each other
  action_constituents(Act,Constituents,Constituent_Constraints),
  % gather allen constraints about the occurance of Act
  action_boundary_constraints(Act,Act_Constraints),
  append(Constituent_Constraints,Act_Constraints,Constraints),
  % compute the sequence graph
  write('      [parser.pl] '), write('Loading action '), write_concept(Act), nl,
  esg_truncated(Act,Constituents,Constraints,[Sequence,
                           PreConditions, PostConditions]),
  write('      [parser.pl] '),
  write_concept(Act), write(' -> '), esg_write(Sequence), nl,
  % assert to Prolog KB
  assertz(parser_grammar(Parser,Act,[Sequence,PreConditions,PostConditions])).

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
  {esg_peak(G_0,Endpoint)}) :- !.

% FIXME: causes a warning?!?
expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_0,
  {'peak_token'(Token)},
  peak_token(Token)) :- !.

expand_esg_term(_P,G_0->G_1,S_0->S_0,A_0->A_0,
  {'pop'(Endpoint)},
  {esg_pop(G_0,Endpoint,G_1)}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,A_0->A_1,
  {'preceded_by'(Act,Pre,Confidence)},
  {activity_preceded_by(Act,Pre,S_0,A_0->A_1,Confidence)}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,Actors,
  {'proceed_with'(Act,Endpoint,Participants)},
  {proceed_with_participants(Act,Endpoint,Actors,Participants)}) :- !.

expand_esg_term(_P,G_0->G_0,S_0->S_0,Actors,
  {'proceed_without'(Act,Endpoint,Participants)},
  {proceed_without_participants(Act,Endpoint,Actors,Participants)}) :- !.
  

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

update_states(States_0->States_1,tok(_Time,Event,-(EvtType),Participants)) :- !,
  set_entity_states(States_0->States_1,Participants,[EvtType,Event,Participants]).
update_states(States_0->States_1,tok(_Time,Event,+(EvtType),Participants)) :- !,
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
%%%%%%%%% activity context

% TODO: do this with annotations in ontology
%   - what if multiple types are context?
%   - what if no context type specified?
context_type(Act,Type) :-
  rdfs_subclass_of(Act,actions:'ComplexPhysicalAction'),!,
  rdf_equal(Type,dul:'PhysicalArtifact').
context_type(Act,Type) :-
  rdfs_subclass_of(Act,actions:'Actuating'),!,
  rdf_equal(Type,ease:'PhysicalArtifact').
context_type(Act,Type) :-
  rdfs_subclass_of(Act,actions:'Grasping'),!,
  rdf_equal(Type,ease:'PhysicalEffector').
context_type(Act,Type) :-
  rdfs_subclass_of(Act,actions:'ReleasingGrasp'),!,
  rdf_equal(Type,ease:'PhysicalEffector').

context_entities(Actors_0->Actors_1,Activity) :-
  % filter out any actors which are no potential candidate for
  % being activity context
  context_type(Activity,CtxType),!,
  filter_actors(Actors_0->Actors_1,CtxType).
context_entities(_->[],_).

context_test(A_0,A_1,Act) :-
  context_entities(A_0->Ctx_0,Act),
  context_entities(A_1->Ctx_1,Act),
  flatten_actors(Ctx_0, X_0),
  flatten_actors(Ctx_1, X_1),
  once((
    ( X_0=[] ; X_1=[] ) ;
    ( member(Needle,X_0),
      member(Needle,X_1)))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% activity actors

filter_actors([]->[],_EntityType) :- !.
filter_actors([A0|As0]->Out,EntityType) :-
  filter_actors_endpoint(A0->A1,EntityType),
  filter_actors(As0->As1,EntityType),
  ( A1=[] -> Out=As1 ; Out=[A1|As1] ).

filter_actors_endpoint([]->[],_EntityType) :- !.
filter_actors_endpoint([E0|Es0]->Out,EntityType) :-
  filter_actors_participants(E0->E1,EntityType),
  filter_actors_endpoint(Es0->Es1,EntityType),
  ( E1=[] -> Out=Es1 ; Out=[E1|Es1] ).

filter_actors_participants([]->[],_EntityType) :- !.
filter_actors_participants([X0|Xs0]->[X0|Xs1],EntityType) :-
  rdfs_individual_of(X0,EntityType), !,
  filter_actors_participants(Xs0->Xs1,EntityType).
filter_actors_participants([_|Xs0]->Xs1,EntityType) :-
  filter_actors_participants(Xs0->Xs1,EntityType).


flatten_actors(X,Set) :-
  flatten_actors_(X,[]->Flat),
  list_to_set(Flat,Set).
flatten_actors_([],Flat_0->Flat_0) :- !.
flatten_actors_([X|Rest],Flat_0->Flat_n) :-
  ( is_list(X) ->
    flatten_actors_(X,Flat_0->Flat_1) ;
    Flat_1=[X|Flat_0] ),
  flatten_actors_(Rest,Flat_1->Flat_n).
  

restrict_actors(A0->A0,[]) :- !.
restrict_actors(Actors,EA_new) :-
  restrict_actors_(Actors,EA_new).
restrict_actors_([]->[],_) :- !.
restrict_actors_([EA_0|Xs_0]->Out,EA_new) :-
  restrict_endpoint_actors(EA_0->EA_1,EA_new),
  restrict_actors_(Xs_0->Xs_1,EA_new),
  ( EA_1=[] -> Out=Xs_1 ; Out=[EA_1|Xs_1] ).

restrict_endpoint_actors([]->[], _) :- !.
restrict_endpoint_actors(EA_0->EA_1, EA_new) :-
  flatten_actors(EA_new, Actors_new),
  findall(X, (
    member(X,EA_0),
    once((member(X_actor,Actors_new),
          member(X_actor,X)))),
    Xs),
  ( Xs=[] -> EA_1=EA_0 ; EA_1=Xs ).


proceed_with_actors(_Act,[]->A_selected,A_selected) :- !.
proceed_with_actors(Act,A_0->A_n,A_selected) :-
  % proceed is not possible if the event is about some other
  % contextual thing
  context_test(A_0,A_selected,Act),
  % current actors may have options that
  % can be restricted by the selection
  restrict_actors(A_0->A_1,A_selected),
  % append A_selected as new endpoints
  append(A_selected,A_1,A_n).

proceed_with_participants(Act,_Endpoint,Actors,Participants) :-
  proceed_with_actors(Act,Actors,[[Participants]]).


proceed_without_participants(Act,_Endpoint,A_0->A_0,Participants) :-
  % don't skip if next token has matching context
  \+ context_test(A_0,[[Participants]],Act).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

endpoint_polarization(-(_),-(_)).
endpoint_polarization(+(_),+(_)).

endpoint_state_actors(States_0,Endpoint,Actors_e) :-
  endpoint_type(Endpoint,EventType),
  % find set of events in which typed endpoint occured
  findall(Event_participants, (
    entity_state(States_0,_Entity,EventType,
        [Event,Event_participants]),
    endpoint_polarization(Event,Endpoint)),
    AllParticipants),
  list_to_set(AllParticipants,Actors_e).

activity_preceded_by(Act,Pre,
    States_0,
    Actors_0->Actors_1,
    Confidence) :-
  % check how many endpoints were observed that would sattisfy
  % the pre-conditions
  esg_endpoints(Pre,PreEndpoints),
  ( PreEndpoints=[] -> (
    Actors_1=Actors_0,
    Confidence=1.0
  );(
    findall(Actors_e, (
      member(Endpoint,PreEndpoints),
      endpoint_state_actors(States_0,Endpoint,Actors_e),
      Actors_e \= []
    ), Actors_pre),
    %% confidence
    length(PreEndpoints,Total),
    length(Actors_pre,Actual),
    Confidence is Actual / Total,
    %%%
    proceed_with_actors(Act,Actors_0->Actors_1,Actors_pre)
  )).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parse a single typed phase endpoint.
phase_endpoint(Act,Endpoint,[phase(Endpoint,Event,Participants)]) :->
  % next token has matching endpoint and context
  [ tok(_Time,Event,Endpoint,Participants) ],
  { 'proceed_with'(Act,Endpoint,Participants) }.

phase_endpoint(Act,Endpoint,ParseTree) :->
  % skip tokens of some other activity context
  [ tok(_Time,_Event,Endpoint1,Participants) ],
  { 'proceed_without'(Act,Endpoint1,Participants) },
  phase_endpoint(Act,Endpoint,ParseTree).

phase_endpoint(_Act,Endpoint,[skipped]) :->
  % skip ESG endpoints, i.e., endpoints which were not observed.
  % - only allow skipping if next token is not matching endpoint
  % TODO: only not skip if same type and same context
  { 'peak_token'(tok(_,_,Endpoint1,Participants)) },
  { Endpoint1 \= Endpoint }.

constituent(Parent,-(Act),[action(Act,Term,Confidence)]) :->
  { is_action_endpoint(Act) },
  sub_activity(Parent,action(Act,Term,Confidence)).

constituent(Parent,Endpoint,PhaseTerm) :->
  { is_state_endpoint(Endpoint) ;
    is_process_endpoint(Endpoint) },
  phase_endpoint(Parent,Endpoint, PhaseTerm),
  { 'pop'(Endpoint) }.

% parse endpoints of action constituents
constituents(_,[])   :-> { 'esg'([]->[]) }.  % stop if graph is empty
constituents(Act,[]) :-> { 'peak'(+(Act)) }. % stop at +(Act0) endpoint
constituents(Act,[X|Xs]) :->
  { 'peak'(Endpoint) },
  { \+ endpoint_type(Endpoint,Act) },
  constituent(Act,Endpoint,[X]),
  constituents(Act,Xs).


% parse {PreConditions}[-(Act),...,+(Act)]
activity(action(Act,Constituents,Confidence),[Pre,_Post]) :->
  { 'pop'(-(Act)) },
  { 'preceded_by'(Act,Pre,PreConfidence) },
  constituents(Act,X),
  { delete(X,skipped,Constituents) },
  { Constituents \= [] },
  { activity_confidence(Act,Constituents,PreConfidence,Confidence) },
  % TODO: can we handle post conditions? e.g. surface contact after dropping.
  %       these are ignored at the moment. maybe these could be used to imply tokens which were not observed.
  %{ 'post_conditions'(Act,_Post) },
  { 'pop'(+(Act)) }.


sub_activity(P,ESG_0->ESG_n,States,A_0->A_n,
             Parent,action(Act,Term,Confidence)) -->
  { parser_grammar_(P,Act,[Act_ESG|ActConditions]) },
  { esg_join(ESG_0,[Act,Act_ESG],ESG_1) },
  % parent actions may restrict what actors are used in sub-actions
  { context_entities(A_0->A_1,Parent) },
  activity(P,ESG_1->ESG_n,States,A_1->A_x,
           action(Act,Term,Confidence),
           ActConditions),
  % sub-activities may specify the activity context of parent actions
  { context_entities(A_x->A_y,Parent) },
  { proceed_with_actors(Parent,A_0->A_n,A_y) }.


% parse [*,-(Act),...,+(Act),*]
% TODO idea: re-start parser after each composite action
%        - implement "parsing" states, or prepend state tokens
%        - then yield only actions up to some token, which are complete then?
%        - could be hard to recoverparser state
%           - tokens,graph,states,actors
%           - diffeent graphs used, at different stages,and different actors
%           - also at different stages in token sequence
%        - can re-parsing allprevious tokens be avoided?
some_activity(Parser, Act, States, action(Act,ActTerm,Confidence)) -->
  { parser_grammar_(Parser,Act,[Graph|ActConditions]) },
  activity(Parser,Graph->[],States,[]->_,
           action(Act,ActTerm,Confidence),
           ActConditions).
some_activity(Parser, Act, States_0->States_n, Action) -->
  [T], % skip token
  { update_states(States_0->States_1,T) },
  some_activity(Parser, Act, States_1->States_n, Action).

%% detect_activity(+Parser,+Tokens,-Interpretation).
%
% Detects activities in given token sequence.
%
% @param Parser Parser id.
% @param Tokens Token sequence.
% @param Interpretation An interpretation of the activity represented by the tokens.
%
%detect_activity(Parser,Tokens,Output) :-
  %debug_list_size('the number of input tokens is', Tokens),
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
  debug_list_size('the number of input tokens is', Tokens),
  findall(ActionTerm,
    phrase(some_activity(Parser,_,states{}->_,ActionTerm), Tokens, _),
    Unfiltered),
  filter_detections(Unfiltered,[]->Output),
  debug_list_size('the number of detected actions is',Output),
  debug_parse_tree(Output).

% TODO to be removed
detect_activity2(Parser,Tokens,Interpretation) :-
  detect_activity(Parser,Tokens,ActionTerms),
  findall([L0,L1,X], (
    phrase(interpretation([]->E,[]->X),ActionTerms,[]),
    length(E,L0_), L0 is -L0_,
    length(X,L1)
  ), Xs),
  debug_list_size('the number of combinations is',Xs),
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
term_endpoint(action(_,Xs,_),E) :-
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
detection_contains([action(Act,Xs,_)|_], action(Act,Ys,_)) :-
  forall(member(Y,Ys), detection_contains(Xs,Y)), !.
detection_contains([action(_,Xs,_)|_],Inner) :-
  detection_contains(Xs,Inner), !.
detection_contains([_|Rest],Inner) :-
  detection_contains(Rest,Inner), !.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% confidence

activity_confidence(Act,DetectedConstituents,PreConfidence,Confidence) :-
  event_constituents(Act,AllConstituents,_),
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
activity_confidence(action(Act,_,Confidence),Act,Confidence) :- !.

accumulate_confidence([], 1) :- !.
accumulate_confidence(Xs, Confidence) :-
  findall(C, (
    member(X,Xs),
    activity_confidence(X,_,C)),
    Cs),
  length(Cs,Cs_length),
  sumlist(Cs,Cs_sum),
  Confidence is Cs_sum / Cs_length.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% debugging

debug_parse_tree([]) :- !.
debug_parse_tree([X|Xs]) :-
  debug_parse_tree_('',X),
  debug_parse_tree(Xs).
debug_parse_tree_(Space,action(Act,Xs,Confidence)) :-
  atom_concat(Space,'  ',NextSpace),
  rdf_split_url(_,ActName,Act),
  debug(activity_parser, '~waction(~w,~f)', [Space,ActName,Confidence]),
  ( Xs=[action(_,_,_)|_] ->
  ( forall(member(X,Xs), debug_parse_tree_(NextSpace,X)) ) ;
  ( findall(Y, (
      member(X,Xs),
      phase_debug_atom(X,Y)),
      Ys),
    atomic_list_concat(Ys,',',Endpoints),
    debug(activity_parser, '~w[~w]', [NextSpace,Endpoints])
  )).
phase_debug_atom(phase(-(E),_,_),Atom) :-
  rdf_split_url(_,TypeName,E),
  term_to_atom(-(TypeName),Atom),!.
phase_debug_atom(phase(+(E),_,_),Atom) :-
  rdf_split_url(_,TypeName,E),
  term_to_atom(+(TypeName),Atom),!.

debug_list_size(Msg,L) :-
  length(L,S),
  debug(activity_parser, '[activity_parser] ~w ~d', [Msg, S]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Unit tests

:- begin_tests(activity_parser).
:- dynamic test_parser/1.

test('parser_assert') :-
  parser_create(Parser),
  assertz(test_parser(Parser)),
  rdf_assert('TestTable', rdf:type, dul:'PhysicalArtifact'),
  rdf_assert('TestObject', rdf:type, dul:'PhysicalArtifact'),
  rdf_assert('TestHand', rdf:type, ease:'PhysicalEffector').

% FIXME: Opening/Closing is skipped!!!
%          -> do not skip if some state constituent exists

test('detect_activity(PickingUp)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0, a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0, b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0, c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0, d, +(motions:'GraspingMotion'),           ['TestHand'])
    ],
    [_,action(actions:'PickingUp',_,_)]).

test('detect_activity(not PickingUp)', [fail]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(0.5,a, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,d, +(motions:'GraspingMotion'),           ['TestHand'])
    ],
    [_,action(actions:'PickingUp',_,_)]).

test('detect_activity(GraspLift)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,a,-(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b,-(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c,-(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,d,+(motions:'GraspingMotion'),           ['TestHand']),
    tok(5.0,f,+(force_dynamics:'SupportingContact'), ['TestTable','TestObject'])
    ],
    [_,action(actions:'GraspLift',_,_)]).

test('detect_activity(Placing)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(0.9,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(3.0,c, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,b, +(motions:'ReleasingMotion'),          ['TestHand'])
    ],
    [_,action(actions:'Placing',_,_)]).

test('detect_activity(PickPlace)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,b, +(motions:'GraspingMotion'),           ['TestHand']),
    tok(5.0,a, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(8.0,f, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(9.0,g, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(9.5,c, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(10.0,g,+(motions:'ReleasingMotion'),          ['TestHand'])
    ],
    [_,action(actions:'PickPlace',_,_)]).

test('detect_activity(PickPlace2)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    %%%% first
    tok(0.0,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,b, +(motions:'GraspingMotion'),           ['TestHand']),
    tok(5.0,a, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(8.0,f, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(9.0,g, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(9.5,c, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(10.0,g,+(motions:'ReleasingMotion'),          ['TestHand']),
    %%%% second
    tok(21.0,h, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(23.0,l, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(24.0,h, +(motions:'GraspingMotion'),           ['TestHand']),
    tok(25.0,f, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(28.0,z, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(29.0,u, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(29.5,l, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(30.0,u, +(motions:'ReleasingMotion'),          ['TestHand'])
    ],
    [_,action(actions:'PickPlace',_,_),action(actions:'PickPlace',_,_)]).

test('parser_retract') :-
  test_parser(Parser),
  retractall(parser_grammar(Parser,_,_)),
  rdf_retractall('TestTable',_,_),
  rdf_retractall('TestObject',_,_),
  rdf_retractall('TestHand',_,_).

:- end_tests(activity_parser).
