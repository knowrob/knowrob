:- module(activity_parser,
    [ parser_create/1,
      parser_create/2,
      parser_run/3,
      parser_start/1,
      parser_stop/1,
      parser_stop/2,
      parser_push_token/2,
      parser_pop_finalized/2,
      parser_intermediate_results/2,
      parser_jsonify/2,
      ros_push_token/2
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
:- use_module(library('logging')).
:- use_module(library('http/json')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs'),
    [ rdfs_individual_of/2, rdfs_subclass_of/2 ]).

:- use_module('interval',
	[ interval_constraint/3 ]).
:- use_module('esg').

:- rdf_meta endpoint_type_(t,r),
            parser_grammar_(?,r,r,t),
            parser_create_grammar_(+,r),
            parser_create(-,t),
            parser_run(+,t,t),
            parser_start(+),
            parser_stop(+),
            parser_stop(+,-),
            parser_push_token(+,t).

:- dynamic parser_grammar_/4, % Parser, Workflow, Task Concept, Sequence Graph
           parser_queue_/3,
           parser_subscriber_/2,
           composer_queue_/2,
           parser_thread_/2,
           parser_sub_thread_/3,
           composer_result_/2,
           composer_finalized_/2,
           composer_thread_/2.

%%
parser_info_(Msg) :- log_info(Msg).

%% parser_create(-Parser) is det.
%
% Creates a new activity detection parser.
% For this, sequence graphs are created for each
% workflow known.
%
% @param Parser The id of a activity parser.
%
parser_create(Parser) :-
  findall(WF,
    rdfs_individual_of(WF,dul:'Workflow'),
    Workflows),
  parser_create(Parser,Workflows).

%% parser_create(-Parser,+Workflows) is det.
%
% Creates a new activity detection parser.
% For this, sequence graphs are created for each
% workflow provided.
%
% @param Parser The id of a activity parser.
% @param Workflows A list of workflow IRIs.
%
parser_create(Parser,Workflows) :-
  parser_unique_id_(Parser),
  forall(member(WF,Workflows),(
    parser_create_grammar_(Parser,WF);
    log_warn(invalid_grammar(WF))
  )),
  assertz(composer_result_(Parser,[])),
  assertz(composer_finalized_(Parser,[])),
  mutex_create(Parser).
           
%% generate unique parser id
parser_unique_id_(Parser) :-
  ground(Parser),!,
  \+ parser_grammar_(Parser,_,_,_).
parser_unique_id_(Parser) :-
  random_id_(P),
  ( parser_grammar_(P,_,_,_) ->
    parser_unique_id_(Parser) ;
    Parser = P ).

%%
parser_task_type_(Tsk,TskType) :-
  rdfs_individual_of(Tsk,TskType),
  \+ rdf_equal(TskType, owl:'NamedIndividual').
parser_task_type_(Tsk,Tsk).

%%
parser_create_grammar_(Parser,WF) :-
  rdf_has(WF,soma:isPlanFor,Tsk),
  once(parser_task_type_(Tsk,TskType)),
  % find constituents and their relation to each other
  % TODO: put all of these underneath a shared super property
  findall(S, (
    rdf_has(WF, dul:describes, S) ;
    rdf_has(WF, dul:definesTask, S) ;
    rdf_has(WF, soma:hasStep, S) ;
    rdf_has(WF, soma:hasFirstStep, S) ;
    rdf_has(WF, soma:definesProcess, S)
  ), Steps),
  list_to_set(Steps,Steps0),
  findall(C, (
    member(X,Steps0),
    interval_constraint(X,C,Other),
    once(member(Other,Steps0))
  ), Constraints),
  list_to_set(Constraints,Constraints0),
  % compute the sequence graph
  parser_info_(loading_grammar(WF)),
  esg_truncated(Tsk,Steps0,Constraints0,[Sequence,
                           PreConditions, PostConditions]),
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
    log_warn(no_grammar_(Parser)),
    fail.
no_grammar_(Parser,Tsk) :-
    log_warn(no_grammar_(Parser,Tsk)),
    fail.

%% parser_run(+Parser,+Tokens,-ActTerm) is nondet.
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
  %% start parser thread
  parser_start(Parser),
  %% push tokens to its message queue
  forall(member(T,Tokens),parser_push_token(Parser,T)),
  %% wait and get all results
  parser_stop(Parser,Output).

%% parser_start(+Parser) is det.
%
% Starts a parser, putting it in a state where
% it waits for tokens to be streamed to the message
% queue of the parser thread.
%
% @param Parser The id of a activity parser.
%
parser_start(Parser) :-
  %% message queues used to comminute parse results
  message_queue_create(In),
  message_queue_create(Out),
  message_queue_create(Composed),
  assertz(parser_queue_(Parser,In,Out)),
  assertz(composer_queue_(Parser,Composed)),
  composer_set_intermediate_(Parser,[]),
  %% obtain tokens from ROS topic '/parser/token'
  ignore(ros_subscribe('/parser/token',
      'knowrob/EventToken',
      activity_parser:ros_push_token(Parser),
      Subscriber)),
  assertz(parser_subscriber_(Parser,Subscriber)),
  %% run the parser
  thread_create(parser_run_(Parser),Thread),
  assertz(parser_thread_(Parser,Thread)),
  %% run composer
  thread_create(activity_composer_run_(Parser),ComposeThread),
  assertz(composer_thread_(Parser,ComposeThread)),
  %%
  parser_info_(started(Parser)).

%% the time of the oldest token still active
parser_time_(Parser,Time) :-
  findall(T0, (
    parser_sub_thread_(Parser,Thread,T0),
    is_active_thread_(Thread)
  ), Stamps),
  min_list(Stamps,Time).

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
  %% stop listining to ROS topics
  parser_subscriber_(Parser,Subscriber),
  ros_unsubscribe(Subscriber,'/parser/token'),
  retractall(parser_subscriber_(Parser,_)),
  %% wait until parser thread exits
  parser_stop_threads_(Parser),
  %% and destroy queues
  parser_queues_destroy_(Parser),
  %%
  parser_info_(stopped(Parser)).

parser_stop(Parser,Outputs) :-
  %% wait until parser threads have finished
  parser_stop_threads_(Parser),
  %% get composed actions
  parser_pop_finalized(Parser,Outputs),
  %% and destroy queues
  parser_queues_destroy_(Parser),
  %%
  parser_info_(stopped(Parser)).

%%
parser_stop_threads_(Parser) :-
  ( parser_thread_(Parser,Thread) ->
    parser_join_thread_(Thread) ;
    true
  ),
  forall( parser_sub_thread_(Parser,SubThread,_),
          parser_join_thread_(SubThread)
  ),
  retractall(parser_sub_thread_(Parser,_,_)),
  retractall(parser_thread_(Parser,_)),
  %% join ComposeThread
  composer_thread_(Parser,ComposeThread),
  parser_queue_(Parser,_,OutputQueue),
  thread_send_message(OutputQueue,end_of_file),
  thread_join(ComposeThread),
  retractall(composer_thread_(Parser,_)).

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
  composer_queue_(Parser,Composed),
  message_queue_destroy(Composed),
  retractall(parser_queue_(Parser,_,_)),
  retractall(composer_queue_(Parser,_)).

%% action_parser_push_token(+Parser,+Token) is semidet.
%
% Used to stream tokens into the parser thread.
% Will fail in case no parser thread is running.
%
% @param Parser The id of an activity parser.
% @param Token A event endpoint token.
%
parser_push_token(Parser,Token) :-
  ( is_token_(Token) ; (
    log_warn(invalid_token(Token)),
    fail
  )),!,
  parser_thread_(Parser,Thread),
  is_active_thread_(Thread),
  thread_send_message(Thread,Token).

%%
ros_push_token(Parser,Tok) :-
  %% read input
  get_dict(timestamp,    Tok, Time),
  get_dict(polarization, Tok, Polarization),
  get_dict(event_type,   Tok, EventType),
  get_dict(participants, Tok, Objects),
  %%
  atom_string(EventType_atom,EventType),
  findall(OA, (
    member(O,Objects),
    atom_string(OA,O)
  ), Object_Atoms),
  %%
  ( Polarization = 0 ->
    Endoint = -(EventType_atom);
    Endoint = +(EventType_atom)
  ),
  %%
  parser_push_token(Parser,tok(Time,Endoint,Object_Atoms)).

is_token_(tok(Time,_Endpoint,_Objects)) :- number(Time).

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
  Tok=tok(Time,_,_),
  % token may indicate the start of a new action verb
  forall(
    parser_get_grammar_(Parser,WF,Tsk,[Graph|ActConditions]),
    verb_thread_create(Parser,States,[WF,Tsk,Graph,ActConditions],Time)
  ),
  % add token to each active thread
  forall(
    ( parser_sub_thread_(Parser,VerbThread,_),
      is_active_thread_(VerbThread)
    ),
    thread_send_message(VerbThread,Tok)
  ).

verb_thread_create(Parser,States,Grammar,Time) :-
  thread_create(parse_verb_(Parser,States,Grammar),Thread),
  assertz(parser_sub_thread_(Parser,Thread,Time)).

%%
parse_verb_(Parser,States,[WF,Tsk,Graph,ActConditions]) :-
  parser_queue_(Parser,ParserIn,_),
  %% Create a lazy list of tokens streamed into the message
  %% queue of the thread.
  thread_self(Thread),
  %gtrace,
  setup_call_cleanup(
    lazy_list(lazy_message_queue(Thread, []), Tokens),
    %% Run the parser
    ( phrase(action_(
        [Graph,States,[]]->[[],_,_],
        action(WF,Tsk,Constituents),
        ActConditions),Tokens,_)
    -> thread_send_message(ParserIn,action(WF,Tsk,Constituents))
    ;  true
    ),
    %%
    retractall(parser_sub_thread_(Parser,Thread,_))
  ).

%% wait until all threads have consumed the last token.
%% this is indicated by the thread message queue being empty.
parser_synch_(Parser) :-
  forall(
    parser_sub_thread_(Parser,Thread,_),
    parser_synch__(Thread)
  ).
parser_synch__(Thread) :- \+ is_active_thread_(Thread), !.
parser_synch__(Thread) :- \+ thread_peek_message(Thread,_), !.
parser_synch__(Thread) :-
  % wait for above conditions to be met
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
  { forall( parser_sub_thread_(Parser,Thread,_),
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
  { esg_pop(G2,E1,G3),
    concept_endpoint_(E1,+(Tsk))
  }.

%%
% TODO: there is always also a thread that detects the sub-action starting at the current token.
%       It would be good to wait on this thread here, and to integrate its result as sub-action.
sub_action_([G0,S0,A0]->[G_n,S_n,A_n],
             Parent_WF,action(WF,Tsk,Constituents)) -->
  { parser_get_grammar_(_,WF,Tsk,[G1|TskConditions]),
    once(rdf_has(WF,soma:isPlanFor,Tsk_1)),
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
  constituent_([G0,S0,A0]->Ctx1, WF, E, Endpoint, [X]),
  constituents_(Ctx1->Ctx2, WF, Tsk, Xs).

%
constituent_(Ctx0->Ctx1, WF, _, -(Tsk), [action(SubWF,Tsk,Term)]) -->
  { endpoint_type_(Tsk,dul:'Task') },
  sub_action_(Ctx0->Ctx1, WF, action(SubWF,Tsk,Term)).
constituent_(Ctx0->[G2,S1,A1], _WF, E, Endpoint, PhaseTerm) -->
  { endpoint_type_(Endpoint,soma:'StateType') ;
    endpoint_type_(Endpoint,soma:'ProcessType')
  },
  phase_endpoint_(Ctx0->[G1,S1,A1], E, Endpoint, PhaseTerm),
  { esg_pop(G1,E,G2),
    concept_endpoint_(E,Endpoint)
  }.

% parse a single typed phase endpoint.
phase_endpoint_([G0,S0,A0]->[G0,S2,A1],
    E, Endpoint, [phase(Time,Endpoint,Participants)]) -->
  % next token has matching endpoint and context
  [ Tok ],
  { Tok=tok(Time,Endpoint,Participants),
    update_states_(S0->S1,Tok),
    bind_actors_(E,A0->A1,Participants),
    set_has_token_(S1->S2)
  }.

phase_endpoint_([G0,S0,A0]->Ctx1,E,Endpoint,ParseTree) -->
  % skip tokens of some other activity context
  [ Tok ],
  { Tok=tok(_Time,_IgnoredEndpoint,Participants),
    ignore_actors_(A0,S0,Participants),
    update_states_(S0->S1,Tok)
  },
  phase_endpoint_([G0,S1,A0]->Ctx1,E,Endpoint,ParseTree).

		 /*******************************
		 *	Activity composition	*
		 *******************************/

%%
activity_composer_run_(Parser) :-
  % pop next messsage, wait on it
  parser_queue_(Parser,_,OutQueue),
  thread_get_message(OutQueue,T0),
  parser_intermediate_results(Parser,I0),
  % materialize rest of queue
  message_queue_materialize_(OutQueue,Rest),
  %%
  composer_add_all_(I0->I1,[T0|Rest]),
  composer_finalize_(Parser,I1,I2),
  composer_set_intermediate_(Parser,I2),
  ( member(end_of_file,[T0|Rest])
  -> true
  ;  activity_composer_run_(Parser)
  ).

%%
composer_add_all_(I->I,[]) :- !.
composer_add_all_(I->I,[end_of_file|_]) :- !.
composer_add_all_(I0->I2,[X|Xs]) :-
  activity_composer_add_(I0,X,I1),
  composer_add_all_(I1->I2,Xs).

%% split pending results into ongoing & finalized
composer_finalize_(Parser,Results,Ongoing) :-
  parser_time_(Parser,Time) ->
  composer_finalize__(Parser,Time,Results,Ongoing);
  ( forall(
      member(X,Results),
      composer_push_finalized_(Parser,X)
    ),
    Ongoing=[]
  ).

composer_finalize__(_Parser,_Time,[],[]).
composer_finalize__(Parser,Time,[X|Xs],Os) :-
  term_endtime_(X,EndTime),
  EndTime < Time, !,
  composer_push_finalized_(Parser,X),
  composer_finalize__(Parser,Time,Xs,Os).
composer_finalize__(Parser,Time,[X|Xs],[X|Os]) :-
  composer_finalize__(Parser,Time,Xs,Os).

%% parser_pop_finalized(+Parser,-Outputs) is det.
%
% Empties the output message queue of the activity
% parser. Outputs is a list of activity compositions
% considered as *final* by the parser, i.e., no new token
% could be pushed that would render the items in the list invalid.
%
% @param Parser The id of a activity parser.
% @param Outputs The pending finalized results of the parser.
%
parser_pop_finalized(Parser,Finalized) :-
  composer_queue_(Parser,Q),
  message_queue_materialize_(Q,Finalized).

%%
composer_push_finalized_(Parser,X) :-
  X=action(_,Tsk,_),
  parser_info_(composed(Tsk)),
  %%
  composer_queue_(Parser,Q),
  thread_send_message(Q,X).

%% parser_intermediate_results(+Parser,-Outputs) is det.
%
% Outputs is a list of activity compositions
% considered as *non-final* by the parser, i.e., some new token
% could be pushed that would render the item in the list invalid.
%
% @param Parser The id of a activity parser.
% @param Outputs The intermediate results of the parser.
%
parser_intermediate_results(Parser,Outputs) :-
  with_mutex(Parser,
    composer_result_(Parser,Outputs)
  ).

composer_set_intermediate_(Parser,X) :-
  with_mutex(Parser, (
    retractall(composer_result_(Parser,_)),
    assertz(composer_result_(Parser,X))
  )).

%% add action in case it explains max num of endpoints
activity_composer_add_(I_in,Act,I_out) :-
  Act=action(_,Tsk,_),
  parser_info_(detected(Tsk)),
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
		 *	Serialization		*
		 *******************************/

%% parser_jsonify(+Terms,-JSON_String) is det.
%
% Converts a list of action terms into a JSON-encoded
% atom.
%
% @param Terms A list of action terms.
% @param JSON_String The list as a JSON string.
%
parser_jsonify(Terms,JSON_String) :-
  %%
  findall(Dict, (
    member(X,Terms),
    event_dict_(X,Dict)
  ),ActDicts),
  with_output_to(atom(JSON_String), 
    json_write_dict(current_output, ActDicts)
  ).

%%
event_dict_([],[]) :- !.
event_dict_([X|Xs],[Y|Ys]) :- !,
  event_dict_(X,Y),
  event_dict_(Xs,Ys).

event_dict_(action(Workflow,Tsk,Constituents),
  _{ type: action,
     event_type: EventType,
     event_time: '',
     event_polarization: '',
     description: Descr,
     children: Children,
     participants: []
  }
) :-
  rdf_split_url(_,EventType,Tsk),
  rdf_split_url(_,Descr,Workflow),
  event_dict_(Constituents,Children).

event_dict_(phase(Time,Endpoint,Participants),
  _{ type: phase,
     event_type: EventType,
     event_time: Time,
     event_polarization: Polarization,
     description: '',
     children: [],
     participants: ObjNames
  }
) :-
  endpoint_type(Endpoint,EndpointType),
  rdf_split_url(_,EventType,EndpointType),
  %%
  ( Endpoint=(-(_)) -> Polarization='-' ; Polarization='+' ),
  %%
  findall(N, (
    member(Obj,Participants),
    rdf_split_url(_,N,Obj)
  ),ObjNames).

		 /*******************************
		 *	States		*
		 *******************************/

%%
get_event_state_(States,Obj,EvtType,Participants) :-
  get_dict(Obj,States,ObjStates),
  get_dict(EvtType,ObjStates,Events),
  member(Participants,Events),!.

%%
set_event_states_(S_0,[],_,S_0) :- !.
set_event_states_(S_0,[Obj|Xs],Val,S_n) :-
  set_state_(S_0,Obj,Val,S_1),
  set_event_states_(S_1,Xs,Val,S_n).

set_state_(S_0, Obj, [EvtType,Participants], S_0) :-
  get_event_state_(S_0,Obj,EvtType,PE),
  subset(Participants,PE), !.
set_state_(S_0, Obj,[EvtType,Participants], S_1) :-
  get_or_create_state_(S_0,Obj,states{},ES_0),
  get_or_create_state_(ES_0,EvtType,[],Events_0),
  ( member(Participants,Events_0) ->
    S_1 = S_0 ; (
    put_dict(EvtType,ES_0,[Participants|Events_0],ES_1),
    put_dict(Obj,S_0,ES_1,S_1)
  )).

%%
unset_event_states_(S_0,[],_,S_0) :- !.
unset_event_states_(S_0,[Obj|Xs],Val,S_n) :-
  unset_state_(S_0->S_1,Obj,Val),
  unset_event_states_(S_1,Xs,Val,S_n).

unset_state_(States_0->States_0,Obj,[EvtType,Participants]) :-
  \+ (
    get_event_state_(States_0,Obj,EvtType,PE),
    subset(Participants,PE)
  ), !.

unset_state_(States_0->States_1,Obj,[EvtType,Participants]) :-
  get_dict(Obj,States_0,ES_0),
  get_dict(EvtType,ES_0,Events_0),
  delete(Events_0,Participants,Events_1),
  put_dict(EvtType,ES_0,Events_1,ES_1),
  put_dict(Obj,States_0,ES_1,States_1), !.

%%
get_or_create_state_(States,Key,Default,Value) :-
  once( get_dict(Key,States,Value) ; Value = Default ).

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
  once(rdf_has(R, dul:classifies, only(O_Type))),
  member(O,Objects),
  once(rdfs_individual_of(O, O_Type)),
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
    ( rdf_has(Plan, soma:hasBinding, Binding),
      once((
        rdf_has(Binding, soma:hasBindingRole, X0),
        rdf_has(Binding, soma:hasBindingFiller, X1)
      )),
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
update_states_(S_0->S_1,tok(_Time,-(EvtType),Participants)) :- !,
  set_event_states_(S_0,Participants,[EvtType,Participants],S_1).
update_states_(S_0->S_1,tok(_Time,+(EvtType),Participants)) :- !,
  unset_event_states_(S_0,Participants,[EvtType,Participants],S_1).

%%
ignore_actors_(Bindings,States,Actors) :-
  % make sure the first token was consumed, this is indicated
  % by an empty entry in the states dict.
  get_dict(has_token,States,_),
  % allow skipping endpoint in case none of the
  % participants is bound to the activity context.
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
  % check how many endpoints were observed that would satisfy
  % the pre-conditions
  % FIXME: what about action endpoint pre-conditions? these might fail now
  findall(AE, (
    member(AE,Endpoints),
    AE=(-(_))
  ), ActiveEndpoints),
  findall([Participants,C_Roles], (
    member(Endpoint,ActiveEndpoints),
    endpoint_type(Endpoint,Concept),
    %%
    rdfs_individual_of(Concept,ConceptType),
    get_event_state_(S,_,ConceptType,Participants),
    %%
    concept_roles_(Concept,C_Roles)
  ), ParticipantsAndRoles),
  %% 
  length(ActiveEndpoints,Total),
  length(ParticipantsAndRoles,Actual),
  Total is Actual,
  %%%
  add_bindings_(ParticipantsAndRoles,A0->A1).

%%
concept_roles_(Concept,C_Roles_Set) :-
  findall(CR, (
    rdf_has(Concept,dul:isRelatedToConcept,CR),
    once(rdfs_individual_of(CR,dul:'Role'))
  ),C_Roles),
  list_to_set(C_Roles,C_Roles_Set).

%%
concept_endpoint_(-(X),-(Y)) :- !, rdfs_individual_of(X,Y).
concept_endpoint_(+(X),+(Y)) :- !, rdfs_individual_of(X,Y).

%%
endpoint_type_(E,Type) :-
  endpoint_type(E,Iri),
  rdfs_subclass_of(Iri,Type),!.

%%
term_endpoint(phase(_,E,_),E) :- !.
term_endpoint(action(_,_,Xs),E) :-
  member(X,Xs),
  term_endpoint(X,E).

%%
term_endtime_(phase(Time,_,_),Time) :- !.
term_endtime_(action(_,_,Xs),Time) :-
  findall(T0, (
    member(X,Xs),
    term_endtime_(X,T0)
  ), Stamps),
  max_list(Stamps,Time).

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
