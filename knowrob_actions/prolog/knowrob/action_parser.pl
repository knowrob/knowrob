
:- module(knowrob_action_parser, [
     parser_create/1,
     parser_create/2,
     parser_run/3,
     parser_run/4,
     parser_start/1,
     parser_start/2,
     parser_stop/1,
     parser_stop/2,
     parser_push_token/2,
     parser_pop_term/2,
     parser_peek_term/2
]).
/** <module> Activity parsing.

The main idea of this parser is that activities
can be detected through patterns of force events, states,
and motions that are used to structure the activity.
This structure is described in plans. Hence, a library of plans
can be casted as grammar for the parser.

@author Daniel Beßler
*/

:- use_module(library('debug')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/temporal')). % `interval/2`

:- rdf_meta endpoint_type_(t,r),
            parser_grammar_(?,r,r,t),
            parser_create_grammar_(+,r),
            parser_create(-,t),
            parser_run(+,t,t),
            parser_run(+,t,t,t),
            parser_start(+),
            parser_start(+,t),
            parser_stop(+),
            parser_stop(+,-),
            parser_push_token(+,t),
            parser_pop_term(+,t),
            parser_peek_term(+,t).

:- dynamic parser_grammar_/4, % Parser, Workflow, Task Concept, Sequence Graph
           parser_queue_/3,
           parser_thread_/2,
           parser_sub_thread_/2,
           composer_result_/2,
           composer_thread_/2.

%% parser_create(-Parser) is det.
%% parser_create(-Parser,+Workflows) is det.
%
% Creates a new activity detection parser.
% For this, sequence graphs are created for each
% workflow provided.
%
% @param Parser The id of a activity parser.
% @param Workflows A list of workflow IRIs.
%
parser_create(Parser) :-
  parser_create(Parser,[]).

parser_create(Parser,Workflows) :-
  parser_unique_id_(Parser),
  forall(member(WF,Workflows),(
    parser_create_grammar_(Parser,WF);
    print_message(warning, invalid_grammar(WF))
  )).
           
%% generate unique parser id
parser_unique_id_(Parser) :-
  random_id_(P),
  ( parser_grammar_(P,_,_,_) ->
    parser_unique_id_(Parser) ;
    Parser = P ).

%%
parser_create_grammar_(Parser,WF) :-
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
  assertz(parser_grammar_(Parser,WF,TskType,[Sequence,PreConditions,PostConditions])).

%%
parser_get_grammar_(Parser,WF,Tsk,GraphChild) :-
  var(Tsk), !,
  ( parser_grammar_(Parser,WF,Tsk,GraphChild) *->
    true ; no_grammar_(Parser) ).
parser_get_grammar_(Parser,WF,Tsk,GraphChild) :-
  (( parser_grammar_(Parser,WF,X,GraphChild),
     once(rdfs_subclass_of(Tsk,X)) ) *->
     true ; no_grammar_(Parser,Tsk) ).

no_grammar_(Parser) :-
    print_message(warning, no_grammar_(Parser)),
    fail.
no_grammar_(Parser,Tsk) :-
    print_message(warning, no_grammar_(Parser,Tsk)),
    fail.

%% parser_run(+Parser,+Tokens,-ActTerm) is nondet.
%% parser_run(+Parser,+Tokens,-ActTerm,+Opts) is nondet.
%
% Runs the parser for provided tokens. This blocks until
% the processing has finished.
% Results are bound to the ActTerm argument.
%
% @param Parser The id of a activity parser.
% @param Tokens A list of tokens.
% @param ActTerm The parse result.
% @param Opts A list of parser options.
%
parser_run(Parser,Tokens,Output) :-
  parser_run(Parser,Tokens,Output,[]).
parser_run(Parser,Tokens,Output,Opts) :-
  %% start parser thread
  parser_start(Parser,Opts),
  %% push tokens to its message queue
  forall(member(T,Tokens),parser_push_token(Parser,T)),
  %% wait and get all results
  parser_stop(Parser,ActTerms),
  ( member(compose,Opts) ->
    Output = ActTerms ;
    member(Output,ActTerms)
  ).

%% parser_start(+Parser) is det.
%% parser_start(+Parser,+Opts) is det.
%
% Starts a parser, putting it in a state where
% it waits for tokens to be streamed to the message
% queue of the parser thread.
% Options is a list of configuration parameters.
% Currently only *compose* is a valid option, indicating
% that the parser shall consolidate different detected
% actions into one coherent interpretation.
%
% @param Parser The id of a activity parser.
% @param Opts A list of parser options.
%
parser_start(Parser) :-
  parser_start(Parser,[]).

parser_start(Parser,Opts) :-
  %% message queues used to comminute parse results
  message_queue_create(In),
  message_queue_create(Out),
  assertz(parser_queue_(Parser,In,Out)),
  %% run the parser
  thread_create(parser_run_(Parser),Thread),
  assertz(parser_thread_(Parser,Thread)),
  %% run composer
  ( \+ member(compose,Opts) ; (
    assertz(composer_result_(Parser,[])),
    %%
    thread_create(activity_composer_run_(Parser),ComposeThread),
    assertz(composer_thread_(Parser,ComposeThread))
  )),
  !.

%% parser_stop(+Parser) is det.
%% parser_stop(+Parser,-Outputs) is det.
%
% Stops a parser, sending an *EOF* message to all of
% its threads before joining them.
% This call will also *destroy* the message queues of
% the parser.
% The list of outputs yielded by the parser is unified
% with the Outputs argument.
%
% @param Parser The id of a activity parser.
% @param Outputs The result of the parser.
%
parser_stop(Parser) :-
  %% wait until parser thread exits
  parser_stop_threads_(Parser),
  %% and destroy queues
  parser_queues_destroy_(Parser).

parser_stop(Parser,Outputs) :-
  %% wait until parser thread exits
  parser_stop_threads_(Parser),
  %% materialize the output queue holding remaining results
  parser_queue_(Parser,_,OutputQueue),
  ( composer_thread_(Parser,ComposeThread) -> (
    thread_send_message(OutputQueue,end_of_file),
    thread_join(ComposeThread),
    retractall(composer_thread_(Parser,_)),
    composer_result_(Parser,Outputs)
  ) ; (
    message_queue_materialize_(OutputQueue,Outputs)
  )),
  %% and destroy queues
  parser_queues_destroy_(Parser),
  retractall(composer_result_(Parser,_)).

%%
parser_stop_threads_(Parser) :-
  ( parser_thread_(Parser,Thread) ->
    parser_join_thread_(Thread) ;
    true
  ),
  forall( parser_sub_thread_(Parser,SubThread),
          parser_join_thread_(SubThread)
  ),
  retractall(parser_sub_thread_(Parser,_)),
  retractall(parser_thread_(Parser,_)).

%%
parser_join_thread_(Thread) :-
  \+ is_active_thread_(Thread), !.
parser_join_thread_(Thread) :-
  thread_send_message(Thread,end_of_file),
  thread_join(Thread).

%%
parser_queues_destroy_(Parser) :-
  parser_queue_(Parser,In,Out),
  message_queue_destroy(In),
  message_queue_destroy(Out),
  retractall(parser_queue_(Parser,_,_)).

%% action_parser_push_token(+Parser,+Token) is semidet.
%
% Used to stream tokens into the parser thread.
% Will fail in case no parser thread is running.
%
% @param Parser The id of an activity parser.
% @param Token A event endpoint token.
%
parser_push_token(Parser,Token) :-
  parser_thread_(Parser,Thread),
  is_active_thread_(Thread),
  thread_send_message(Thread,Token).

%% parser_peek_term(+Parser,?ActTerm) is semidet.
%
% Peeks the next result from the parsers' output
% message queue without removing it.
% Fails in case the queue is empty.
%
% @param Parser The id of an activity parser.
% @param ActTerm An action that was detected by the parser.
%
parser_peek_term(Parser,ActTerm) :-
  parser_queue_(Parser,_,OutQueue),
  thread_peek_message(OutQueue,ActTerm).

%% parser_pop_term(+Parser,?ActTerm) is semidet.
%
% Pops the next result from the parsers' output
% message queue.
% Blocks until the queue has the next item.
%
% @param Parser The id of an activity parser.
% @param ActTerm An action that was detected by the parser.
%
parser_pop_term(Parser,ActTerm) :-
  parser_queue_(Parser,_,OutQueue),
  thread_get_message(OutQueue,ActTerm).

%%
parser_run_(Parser) :-
  parser_queue_(Parser,_,OutQueue),
  %% Create a lazy list of tokens streamed into the message
  %% queue of the thread.
  thread_self(Thread),
  lazy_list(lazy_message_queue(Thread, []), Tokens),
  %% Run the parser
  forall(
    phrase(action_threaded_(Parser,states{}->_,ActTerm), Tokens, _),
    thread_send_message(OutQueue,ActTerm)
  ).

%%
push_token_(Parser,States,Tok) :-
  % token may indicate the start of a new action verb
  forall(
    parser_get_grammar_(Parser,WF,Tsk,[Graph|ActConditions]),
    verb_thread_create(Parser,States,[WF,Tsk,Graph,ActConditions])
  ),
  % add token to each active thread
  forall(
    ( parser_sub_thread_(Parser,VerbThread),
      is_active_thread_(VerbThread)
    ),
    thread_send_message(VerbThread,Tok)
  ).

verb_thread_create(Parser,States,Grammar) :-
  thread_create(parse_verb_(Parser,States,Grammar),Thread),
  assertz(parser_sub_thread_(Parser,Thread)).

%%
parse_verb_(Parser,States,[WF,Tsk,Graph,ActConditions]) :-
  parser_queue_(Parser,ParserIn,_),
  %% Create a lazy list of tokens streamed into the message
  %% queue of the thread.
  thread_self(Thread),
  lazy_list(lazy_message_queue(Thread, []), Tokens),
  %% Run the parser
  ( phrase(action_(
        [Graph,States,[]]->[[],_,_],
        action(WF,Tsk,Constituents),
        ActConditions
    ),Tokens,_) ->
    thread_send_message(ParserIn,action(WF,Tsk,Constituents)) ;
    true
  ),
  %%
  retractall(parser_sub_thread_(Parser,Thread)).

%% wait until all threads have consumed the last token
parser_synch_(Parser) :-
  forall(
    parser_sub_thread_(Parser,Thread),
    parser_synch__(Thread)
  ).
parser_synch__(Thread) :- \+ is_active_thread_(Thread), !.
parser_synch__(Thread) :- \+ thread_peek_message(Thread,_), !.
parser_synch__(Thread) :-
  % TODO: is there an event-driven way to wait for a queue to be empty?
  sleep(0.001),
  parser_synch__(Thread).

		 /*******************************
		 *	DCG grammar rules	*
		 *******************************/

%%
action_threaded_(Parser,_States,ActTerm) -->
  % first empty queue of results before pushing next token
  { parser_synch_(Parser) },
  { parser_queue_(Parser,Queue,_),
    message_queue_materialize_(Queue,ActTerms),
    member(ActTerm,ActTerms)
  }.
action_threaded_(Parser,S0->Sn,ActTerm) -->
  % push next token to all threads
  [Tok],
  { update_states_(S0->S1,Tok),
    push_token_(Parser,S1,Tok)
  },
  action_threaded_(Parser,S1->Sn,ActTerm).
action_threaded_(Parser,_States,ActTerm) -->
  % join all thread when *end_of_file* is reached
  [end_of_file],
  { forall( parser_sub_thread_(Parser,Thread),
            parser_join_thread_(Thread) ) },
  { parser_queue_(Parser,Queue,_),
    message_queue_materialize_(Queue,ActTerms),
    member(ActTerm,ActTerms)
  }.

%% parse {PreConditions}[-(Tsk),...,+(Tsk)]
action_([G0,S0,A0]->[G3,S2,A2],
    action(WF,Tsk,Constituents),[Pre,_Post]) -->
  { esg_pop(G0,E0,G1),
    concept_endpoint_(E0,-(Tsk)),
    action_preceded_by_(Pre,S0,A0->A1)
  },
  constituents_([G1,S0,A1]->[G2,S2,A2],WF,Tsk,Constituents),
  % TODO: can we handle post conditions? e.g. surface contact after dropping.
  %       these are ignored at the moment. maybe these could be used to imply tokens which were not observed.
  %{ 'post_conditions'(Tsk,_Post) },
  { esg_pop(G2,E1,G3),
    concept_endpoint_(E1,+(Tsk))
  }.

%%
sub_action_([G0,S0,A0]->[G_n,S_n,A_n],
             Parent_WF,action(WF,Tsk,Constituents)) -->
  { parser_get_grammar_(_,WF,Tsk,[G1|TskConditions]),
    rdf_has(WF,ease:isPlanFor,Tsk_1),
    esg_join(G0,[Tsk_1,G1],G2)
  },
  % assign roles of WF/ACT given bindings from the parent workflow
  { apply_role_bindings_(Parent_WF, Tsk, A0, A0->A1) },
  action_([G2,S0,A1]->[G_n,S_n,A2],
           action(WF,Tsk,Constituents),
           TskConditions),
  % assign roles of parent TSK/WF given role assignments
  % inferred in activity predicate, and bindings from WF.
  % this will fail in case there are conflicting instantiations!
  { apply_role_bindings_(WF, Tsk, A2, A2->A_n) }.

%% parse endpoints of action constituents
constituents_([[],S,A]->[[],S,A], _, []) --> []. % stop if graph is empty
constituents_([G0,S0,A0]->[G0,S0,A0], _WF, Tsk,[]) -->
  % stop at +(Act0) endpoint
  { esg_peak(G0,E),
    concept_endpoint_(E,+(Tsk))
  }.
constituents_([G0,S0,A0]->Ctx2, WF, Tsk,[X|Xs]) -->
  { esg_peak(G0,E),
    concept_endpoint_(E,Endpoint),
    \+ endpoint_type(Endpoint,Tsk)
  },
  constituent_([G0,S0,A0]->Ctx1, WF, Endpoint, [X]),
  constituents_(Ctx1->Ctx2, WF, Tsk, Xs).

%
constituent_(Ctx0->Ctx1, WF, -(Tsk), [action(SubWF,Tsk,Term)]) -->
  { endpoint_type_(Tsk,dul:'Task') },
  sub_action_(Ctx0->Ctx1, WF, action(SubWF,Tsk,Term)).
constituent_(Ctx0->[G2,S1,A1], _WF, Endpoint, PhaseTerm) -->
  { endpoint_type_(Endpoint,ease_state:'Gestallt') ;
    endpoint_type_(Endpoint,ease_proc:'ProcessType')
  },
  phase_endpoint_(Ctx0->[G1,S1,A1], Endpoint, PhaseTerm),
  { esg_pop(G1,E,G2),
    concept_endpoint_(E,Endpoint)
  }.

% parse a single typed phase endpoint.
phase_endpoint_([G0,S0,A0]->[G0,S2,A1],
    Endpoint, [phase(Endpoint,Event,Participants)]) -->
  % next token has matching endpoint and context
  [ Tok ],
  { Tok=tok(_Time,Event,Endpoint,Participants),
    update_states_(S0->S1,Tok),
    esg_peak(G0,Endpoint2), % TODO: why Endpoint2 here?
    bind_actors_(Endpoint2,A0->A1,Participants),
    set_has_token_(S1->S2)
  }.

phase_endpoint_([G0,S0,A0]->Ctx1,Endpoint,ParseTree) -->
  % skip tokens of some other activity context
  [ Tok ],
  { Tok=tok(_Time,_Event,_IgnoredEndpoint,Participants),
    ignore_actors_(A0,S0,Participants),
    update_states_(S0->S1,Tok)
  },
  phase_endpoint_([G0,S1,A0]->Ctx1,Endpoint,ParseTree).

		 /*******************************
		 *	Activity composition	*
		 *******************************/

%%
activity_composer_run_(Parser) :-
  parser_pop_term(Parser,Term),
  ( Term=end_of_file -> true ; (
    composer_result_(Parser,I0),
    activity_composer_add_(I0,Term,I1),
    retractall(composer_result_(Parser,_)),
    assertz(composer_result_(Parser,I1)),
    activity_composer_run_(Parser)
  )).

%%
activity_composer_add_(I_in,Act,I_out) :-
  %%
  findall(E0,
    term_endpoint(Act,E0),
    Endpoints_act
  ),
  %%
  findall(Act0,
    overlapping_action_(I_in,Endpoints_act,Act0),
    ActsOverlap
  ),
  findall(E1, (
    member(Act1,ActsOverlap),
    term_endpoint(Act1,E1)
  ), Endpoints_overlap),
  %%
  list_to_set(Endpoints_act,Endpoint_set0),
  list_to_set(Endpoints_overlap,Endpoint_set1),
  length(Endpoint_set0,Length0),
  length(Endpoint_set1,Length1),
  ( Length0 < Length1 -> I_out=I_in ; (
    subtract(I_in,ActsOverlap,I_tmp),
    I_out=[Act|I_tmp]
  )).

%%
overlapping_action_(I,Endpoints,Act_overlapping) :-
  member(Act_overlapping,I),
  once((
    term_endpoint(Act_overlapping,Needle),
    member(Needle,Endpoints)
  )).

		 /*******************************
		 *	States		*
		 *******************************/

%%
get_event_state_(States,Obj,EvtType,[Event,Participants]) :-
  get_dict(Obj,States,ObjStates),
  get_dict(EvtType,ObjStates,Events),
  member([Event,Participants],Events),!.

%%
set_event_states_(S_0,[],_,S_0) :- !.
set_event_states_(S_0,[Obj|Xs],Val,S_n) :-
  set_state_(S_0,Obj,Val,S_1),
  set_event_states_(S_1,Xs,Val,S_n).

set_state_(S_0, Obj, [EvtType,Event,Participants], S_0) :-
  get_event_state_(S_0,Obj,EvtType,[-(Event),PE]),
  subset(Participants,PE), !.
set_state_(S_0, Obj,[EvtType,Event,Participants], S_1) :-
  get_or_create_state_(S_0,Obj,states{},ES_0),
  get_or_create_state_(ES_0,EvtType,[],Events_0),
  ( member([Event,_],Events_0) ->
    S_1 = S_0 ; (
    delete_previous_events(Events_0->Events_1,Participants),
    put_dict(EvtType,ES_0,[[-(Event),Participants]|Events_1],ES_1),
    put_dict(Obj,S_0,ES_1,S_1)
  )).

%%
unset_event_states_(S_0,[],_,S_0) :- !.
unset_event_states_(S_0,[Obj|Xs],Val,S_n) :-
  unset_state_(S_0->S_1,Obj,Val),
  unset_event_states_(S_1,Xs,Val,S_n).

unset_state_(States_0->States_0,Obj,[EvtType,Event,Participants]) :-
  \+ (
    get_event_state_(States_0,Obj,EvtType,[-(Event),PE]),
    subset(Participants,PE)
  ), !.

unset_state_(States_0->States_1,Obj,[EvtType,Event,Participants]) :-
  get_dict(Obj,States_0,ES_0),
  get_dict(EvtType,ES_0,Events_0),
  delete(Events_0,[-(Event),_],Events_1),
  put_dict(EvtType,ES_0,[[+(Event),Participants]|Events_1],ES_1),
  put_dict(Obj,States_0,ES_1,States_1), !.

%%
get_or_create_state_(States,Key,Default,Value) :-
  once( get_dict(Key,States,Value) ; Value = Default ).

%%
delete_previous_events([]->[],_) :- !.
delete_previous_events([[X,P_X]|Xs]->Events_n,P_0) :-
  % delete events with identical participants!
  % TODO: what is going on here?
  delete_previous_events(Xs->Events_1,P_0),
  ( subset(P_X,P_0) ->
    Events_n=Events_1 ;
    Events_n=[[X,P_X]|Events_1] ).

		 /*******************************
		 *	Role bindings		*
		 *******************************/

%%
bind_actors_(Endpoint,Bindings0->Bindings1,Actors) :-
  % find all roles of the concept associated to an endpoint
  endpoint_type(Endpoint,Concept),
  concept_roles_(Concept,C_Roles),
  % create a binding and try to apply it in current context
  binding_create_(Actors,C_Roles,NewBinding),
  add_binding_(Bindings0->Bindings1,NewBinding).

%%
add_binding_([]->A_new,A_new) :- !.
add_binding_(A_0->A_1,A_new) :-
  apply_role_assignments_(A_new,A_0->A_1).

%%
add_bindings_([],A_0->A_0) :- !.
add_bindings_([[Participants,Roles]|Rest], A_0->A_n) :-
  binding_create_(Participants,Roles,Binding),
  add_binding_(A_0->A_1,Binding),
  add_bindings_(Rest,A_1->A_n).

%%
binding_create_(Objects,Roles,Bindings) :-
  % find object-role pairs
  binding_create__(Objects,Roles,ObjectRoles),
  % merge pairs to object-list(role) pairs
  findall([O,OR], (
    member(O,Objects),
    findall(OR0, member([O,OR0],ObjectRoles), OR),
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

binding_update_([O,OR],[]->[[O,OR]]) :- !.
binding_update_([O,OR],[[O,_]|Xs]->[[O,OR]|Xs]) :- !.
binding_update_([O,OR],[X|Xs]->[X|Ys]) :-
  binding_update_([O,OR],Xs->Ys).

%%
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
apply_role_assignment_([O,O_R], A_0->[[O,O_R]|A_0]).

%%
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

		 /*******************************
		 *	helper			*
		 *******************************/

%%
set_has_token_(S_0->S_0) :-
  get_dict(has_token,S_0,_), !.
set_has_token_(S_0->S_1) :-
  put_dict(has_token,S_0,_{},S_1).

%%
update_states_(S_0->S_1,tok(_Time,_,-(Event),Participants)) :- !,
  ( rdfs_individual_of(Event,owl:'Class') ->
    EvtType = Event ;
    kb_type_of(Event,EvtType)
  ),
  set_event_states_(S_0,Participants,[EvtType,Event,Participants],S_1).
update_states_(S_0->S_1,tok(_Time,_,+(Event),Participants)) :- !,
  ( rdfs_individual_of(Event,owl:'Class') ->
    EvtType = Event ;
    kb_type_of(Event,EvtType)
  ),
  unset_event_states_(S_0,Participants,[EvtType,Event,Participants],S_1).

%%
ignore_actors_(Bindings,States,Actors) :-
  % make sure the first token was consumed, this is indicated
  % by an empty entry in the states dict.
  get_dict(has_token,States,_),
  % allow skipping endpoint in case none of the
  % participants is bound to the activity context.
  % TODO: this is too restrictive, e.g. bumping the object
  %           into another object, dropping it and picking it up again, etc.
  forall(
    member(O,Actors),
    \+ member([O,_],Bindings)
  ).

%%
action_preceded_by_(G,S,A_0->A_1) :-
  esg_endpoints(G,Endpoints),
  Endpoints \= [],!,
  action_preceded_by__(Endpoints,S,A_0->A_1).
action_preceded_by_(_G,_S,A_0->A_0).

action_preceded_by__(Endpoints,S,A0->A1) :-
  % check how many endpoints were observed that would sattisfy
  % the pre-conditions
  findall([Participants,C_Roles], (
    member(Endpoint,Endpoints),
    endpoint_type(Endpoint,Concept),
    %%
    kb_type_of(Concept,ConceptType),
    get_event_state_(S,_,ConceptType, [Evt,Participants]),
    same_polarization_(Evt,Endpoint),
    %%
    concept_roles_(Concept,C_Roles)
  ), ParticipantsAndRoles),
  %% 
  length(Endpoints,Total),
  length(ParticipantsAndRoles,Actual),
  Total is Actual,
  %%%
  add_bindings_(ParticipantsAndRoles,A0->A1).

%%
concept_roles_(Concept,C_Roles_Set) :-
  findall(CR, (
    kb_triple(Concept,dul:isRelatedToConcept,CR),
    kb_type_of(CR,dul:'Role')
  ),C_Roles),
  list_to_set(C_Roles,C_Roles_Set).

%%
concept_endpoint_(-(X),-(Y)) :- !, kb_type_of(X,Y).
concept_endpoint_(+(X),+(Y)) :- !, kb_type_of(X,Y).

%%
endpoint_type_(E,Type) :-
  endpoint_type(E,Iri),
  rdfs_subclass_of(Iri,Type),!.

%%
same_polarization_(-(_),-(_)).
same_polarization_(+(_),+(_)).

%%
term_endpoint(phase(-(_),Event,_),-(Event)) :- !.
term_endpoint(phase(+(_),Event,_),+(Event)) :- !.
term_endpoint(action(_,_,Xs),E) :-
  member(X,Xs),
  term_endpoint(X,E).

%%
random_id_(Id) :-
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Id, Alpha_random).

%%
is_active_thread_(Thread) :-
  is_thread(Thread),
  thread_property(Thread, status(running)).

%%
message_queue_materialize_(Queue,[]) :-
  \+ thread_peek_message(Queue,_), !.
message_queue_materialize_(Queue,[X|Xs]) :-
  thread_get_message(Queue,X),
  message_queue_materialize_(Queue,Xs).
