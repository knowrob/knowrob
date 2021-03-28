:- module(lang_query,
    [ ask(t),        % +Statement
      ask(t,t,t),    % +Statement, +QScope, -FScope
      ask(t,t,t,t),  % +Statement, +QScope, -FScope, +Options
      tell(t),       % +Statement
      tell(t,t),     % +Statement, +Scope
      tell(t,t,t),   % +Statement, +Scope, +Options
      forget(t),     % +Statement
      forget(t,t),   % +Statement, +Scope
      forget(t,t,t), % +Statement, +Scope, +Options
      is_callable_with(?,t),  % ?Backend, :Goal
      call_with(?,t,+)        % +Backend, :Goal, +Options
    ]).
/** <module> Main interface predicates for querying the knowledge base.

@author Daniel Beßler
@license BSD
*/

:- op(1100, xfx, user:(?>)).
:- op(1100, xfx, user:(+>)).
:- op(1100, xfx, user:(?+>)).

:- use_module(library('semweb/rdf_db'),
	[ rdf_global_term/2 ]).
:- use_module('scope',
    [ current_scope/1, universal_scope/1 ]).
:- use_module('mongolog/mongolog').

% interface implemented by query backends
:- multifile is_callable_with/2.
:- multifile call_with/3.

% create a thread pool for query processing
:- worker_pool_create('lang_query').

% the named thread pool used for query processing
thread_pool('lang_query').

%% ask(+Statement) is nondet.
%
% Same as ask/3 with default scope to include
% only facts that hold now.
%
% @param Statement a statement term.
%
ask(Statement) :-
	current_scope(QScope),
	ask(Statement, QScope, _, []).

%% ask(+Statement, +QScope, -FScope) is nondet.
%
% Same as ask/4 with empty options list.
%
% @param Statement a statement term.
%
ask(Statement, QScope, FScope) :-
	ask(Statement, QScope, FScope, []).

%% ask(+Statement, +QScope, -FScope, +Options) is nondet.
%
% True if Statement holds within QScope.
% Statement can also be a list of statements.
% FactScope is the actual scope of the statement being true that overlaps
% with QScope. Options include:
%
%     - max_queue_size(MaxSize)
%     Determines the maximum number of messages queued in each stage.  Default is 50.
%     - graph(GraphName)
%     Determines the named graph this query is restricted to. Note that graphs are organized hierarchically. Default is user.
%
% Any remaining options are passed to the querying backends that are invoked.
%
% @param Statement a statement term.
% @param QScope the requested scope.
% @param FScope the actual scope.
% @param Options list of options.
%
ask(Statements, QScope, FScope, Options) :-
	is_list(Statements),
	!,
	comma_list(Goal, Statements),
	ask(Goal, QScope, FScope, Options).

ask(Statement, QScope, FScope, Options) :-
	% grounded statements have no variables
	% in this case we can limit to one solution here
	ground(Statement),
	!,
	once(ask1(Statement, QScope, FScope, Options)).

ask(Statement, QScope, FScope, Options) :-
	%\+ ground(Statement),
	ask1(Statement, QScope, FScope, Options).

%%
ask1(Goal, QScope, FScope, Options) :-
	option(fields(Fields), Options, []),
	merge_options(
		[ scope(QScope),
		  user_vars([['v_scope',FScope]|Fields])
		],
		Options, Options1),
	% expand query, e.g. replace rule heads with bodies etc.
	% TODO: expansion should not be depending on mongolog
	mongolog_expand(Goal, Expanded),
	% FIXME: not so nice that flattening is needed here
	flatten(Expanded, Flattened),
	ask1(Flattened, Options1).

%%
ask1(SubGoals, Options) :-
	% create a list of step(SubGoal, OutQueue, Channels) terms
	maplist([SubGoal,Step]>>
		query_step(SubGoal,Step),
		SubGoals, Steps),
	% combine steps if possible
	% TODO: use partial results of backends to reduce number of operations
	%       for some cases
	combine_steps(Steps, Combined),
	% need to remember pattern of variables for later unification
	% TODO: improve the way how instantiations are communicated between steps.
	%       currently the same pattern of _all_ variables in the expanded goal
	%       is used in each step. But this is not needed e.g. the input for the
	%       first step could be empty list instead.
	%       An easy optimization would be that each step has a pattern with
	%       variables so far to reduce overall number of elements in comm pattern.
	term_variables(SubGoals, Pattern),
	setup_call_cleanup(
		start_pipeline(Combined, Pattern, Options, FinalStep),
		materialize_pipeline(FinalStep, Pattern, Options),
		stop_pipeline(Combined)
	).


%% tell(+Statement) is nondet.
%
% Same as tell/2 with universal scope.
%
% @param Statement a statement term.
%
tell(Statement) :-
	universal_scope(Scope),
	tell(Statement, Scope, []).

%% tell(+Statement, +Scope) is nondet.
%
% Same as tell/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
tell(Statement, Scope) :-
	tell(Statement, Scope, []).

%% tell(+Statement, +Scope, +Options) is semidet.
%
% Tell the knowledge base that some statement is true.
% Scope is the scope of the statement being true.
% Statement can also be a list of statements. Options include:
%
%     - graph(GraphName)
%     Determines the named graph this query is restricted to. Note that graphs are organized hierarchically. Default is user.
%
% Any remaining options are passed to the querying backends that are invoked.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
% @param Options list of options.
%
tell(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	comma_list(Statement, Statements),
	tell(Statement, Scope, Options).

tell(Statement, Scope, Options) :-
	% ensure there is a graph option
	set_graph_option(Options, Options0),
	% compile and call statement
	(	setting(mng_client:read_only, true)
	->	log_warning(db(read_only(tell)))
	;	mongolog_call(project(Statement), [scope(Scope)|Options0])
	).


%% forget(+Statement) is nondet.
%
% Same as forget/2 with universal scope.
%
% @param Statement a statement term.
%
forget(Statement) :-
	wildcard_scope(Scope),
	forget(Statement, Scope, []).

%% forget(+Statement, +Scope) is nondet.
%
% Same as forget/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
forget(Statement, Scope) :-
	forget(Statement, Scope, []).

%% forget(+Statement, +Scope, +Options) is semidet.
%
% Forget that some statement is true.
% Statement must be a term triple/3. 
% It can also be a list of such terms.
% Scope is the scope of the statement to forget. Options include:
%
%     - graph(GraphName)
%     Determines the named graph this query is restricted to. Note that graphs are organized hierarchically. Default is user.
%
% Any remaining options are passed to the querying backends that are invoked.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
% @param Options list of options.
%
forget(_, _, _) :-
	setting(mng_client:read_only, true),
	!.

forget(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	forall(
		member(Statement, Statements),
		forget(Statement, Scope, Options)
	).

forget(triple(S,P,O), Scope, Options) :-
	% TODO: support other language terms too
	%	- rather map to kb_call(retractall(Term)) or kb_call(unproject(Term))
	% ensure there is a graph option
	set_graph_option(Options, Options0),
	% append scope to options
	merge_options([scope(Scope)], Options0, Options1),
	% get the query document
	mng_triple_doc(triple(S,P,O), Doc, Options1),
	% run a remove query
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, Doc).


		 /*******************************
		 *	    	PIPELINES	  	 	*
		 *******************************/

% start processing all steps of a pipeline
start_pipeline([FirstStep|Rest], Pattern, Options, FinalStep) :-
	% create step message queues
	create_step_queues(FirstStep, Options),
	% send uninstantiated pattern as starting point
	% for the first step.
	forall(
		step_input(FirstStep,InQueue),
		thread_send_message(InQueue,end_of_stream(Pattern))
	),
	start_pipeline1([FirstStep|Rest], Pattern, Options, FinalStep).

%
start_pipeline1([Step|Rest], Pattern, Options, FinalStep) :-
	% create step message queues
	create_step_queues(Step, Options),
	% start processing the step.
	% note that this will not instantiate the pattern
	% as processing is done in a separate thread.
	(	Rest==[] -> true % the last step is processed in materialize_pipeline
	;	start_pipeline_step(Step, Pattern, Options)
	),
	% continue for remaining steps.
	start_pipeline2(Rest, Pattern, Options, Step, FinalStep).

%
start_pipeline2([], _Pattern, _Options, FinalStep, FinalStep) :- !.
start_pipeline2([Step|Rest], Pattern, Options, LastStep, FinalStep) :-
	% use output queue of LastStep to provide input for Step.
	connect_steps(LastStep, Step, Linked),
	start_pipeline1([Linked|Rest], Pattern, Options, FinalStep).

% call pipeline step in multiple backends
start_pipeline_step(
		step(Goal,OutQueue,Channels),
		Pattern, Options) :-
	thread_pool(Pool),
	forall(
		% iterate over different backends
		member([Backend,InQueue], Channels),
		% run call_with/3 in a thread for each backend
		worker_pool_start_work(Pool, InQueue,
			lang_query:call_with(
				Backend, Goal, OutQueue,
				[ input_queue(InQueue),
				  goal_variables(Pattern)
				| Options ]
			)
		)
	).


% retrieve results of a pipeline
materialize_pipeline(
		step(Goal,_,[[Backend,InQueue]]),
		Pattern, Options) :-
	!,
	% call the last step in this thread in case it has a single backend
	call_with(Backend, Goal,
		[ input_queue(InQueue),
		  goal_variables(Pattern)
		| Options ]).

materialize_pipeline(FinalStep, Pattern, Options) :-
	% else start worker thread and poll results from output queue
	start_pipeline_step(FinalStep, Pattern, Options),
	% materialize the output stream of last step in the pipeline.
	% this effectively instantiates the variables in the input query.
	step_output(FinalStep, LastOut),
	message_queue_materialize(LastOut, Pattern).


% stop processing all steps of a pipeline
stop_pipeline([First|Rest]) :-
	stop_pipeline_step(First),
	stop_pipeline(Rest).

stop_pipeline_step(step(_, OutQueue, Channels)) :-
	thread_pool(Pool),
	worker_pool_stop_work(Pool,OutQueue),
	forall(
		member([_,InQueue], Channels),
		worker_pool_stop_work(Pool,InQueue)
	).


% map a goal to a step term
query_step(Goal, step(Goal, _, Channels)) :-
	% note message queues are created later after steps are merged
	% because creating them creates some overhead.
	% TODO: use pool of message queues to avoid creating them each time a goal is called?
	findall([X,_], is_callable_with(X,Goal), Channels),
	Channels \== [].


% ensure message queues associated to a step are instantiated
create_step_queues(step(_, OutQueue, Channels), Options) :-
	option(max_queue_size(MaxSize), Options, 50),
	create_queue_(OutQueue,MaxSize),
	create_step_queues1(Channels,MaxSize).

create_step_queues1([],_) :- !.
create_step_queues1([[_,InQueue]|Rest],MaxSize) :-
	create_queue_(InQueue,MaxSize),
	create_step_queues1(Rest,MaxSize).

%
create_queue_(Queue,_) :- nonvar(Queue),!.
create_queue_(Queue,MaxSize) :-
	% TODO: use alias option. make sure message_queue_destroy is called then.
	message_queue_create(Queue, [max_size(MaxSize)]).

%
step_input(step(_,_,Channels),InQueue) :- member([_,InQueue],Channels).
step_output(step(_,OutQueue,_),OutQueue).


% connect the output of LastStep to the input of ThisStep
connect_steps(
		step(_,OutQueue0,_),              % LastStep
		step(G,OutQueue1,[[Backend,_]]),  % ThisStep
		step(G,OutQueue1,[[Backend,OutQueue0]])) :-
	% if ThisStep has only a single channel,
	% use the output queue of LastStep directly as input
	% queue in ThisStep
	!.

connect_steps(LastStep, ThisStep, ThisStep) :-
	% else feed messages from the output queue of LastStep
	% into the input queues of different channels in a worker thread.
	thread_pool(Pool),
	step_output(LastStep, OutQueue),
	worker_pool_start_work(Pool, OutQueue,
		lang_query:connect_steps(LastStep, ThisStep)).

% send all outeput messages of LastStep to all input queues of ThisStep
connect_steps(LastStep, ThisStep) :-
	step_output(LastStep, OutQueue),
	catch(
		message_queue_materialize(OutQueue, Msg),
		Error,
		Msg=error(Error) % pass any error to next step
	),
	% send msg to every input queue
	forall(
		step_input(ThisStep, InQueue),
		thread_send_message(InQueue, Msg)
	).


% reduce number of query steps by merging consecutive steps if possible
combine_steps([],[]) :- !.
combine_steps([X1,X2|Xs], Combined) :-
	combine_steps(X1,X2,X), !,
	combine_steps([X|Xs], Combined).
combine_steps([X|Xs], [X|Ys]) :-
	combine_steps(Xs, Ys).

% generally allow combining steps that run on the same one backend
combine_steps(
	step(G1,         Out1, [[Backend,In1]]),
	step(G2,         _,    [[Backend,_]]),
	step(','(G1,G2), Out1, [[Backend,In1]])).


		 /*******************************
		 *	  		  BACKENDS 		  	*
		 *******************************/

%% is_callable_with(?Backend, :Goal) is nondet.
%
% True if Backend is a querying backend that can handle Goal.
%
is_callable_with(mongolog, Goal) :-
	% FIXME: HACK: computables are added as mongolog predicates to make expansion work
	%              pull expansion out of mongolog, then do not include computables in is_mongolog_predicate
	\+ is_callable_with(computable, Goal),
	is_mongolog_predicate(Goal).


% call Goal in Backend and send result on OutQueue
call_with(Backend, Goal, OutQueue, Options) :-
	% pass any error to output queue consumer
	catch(
		call_with1(Backend, Goal, OutQueue, Options),
		Error,
		thread_send_message(OutQueue, error(Error))
	).

call_with(_Backend, _Goal, OutQueue, _Options) :-
	% send end_of_stream message needed
	% to terminate worker waiting for next result.
	thread_send_message(OutQueue, end_of_stream).

%
call_with1(Backend, Goal, OutQueue, Options) :-
	option(goal_variables(Pattern), Options),
	% call goal in backend
	call_with(Backend, Goal, Options),
	% publish result via OutQueue
	thread_send_message(OutQueue, Pattern).


%% call_with(+Backend, :Goal, +Options) is nondet.
%
% Calls a goal in given backend.
% The options list may contain additional
% backend specific options.
%
% @param Backend the backend name
% @param Goal a goal term
% @param Options list of options
%
call_with(mongolog, Goal, Options) :-
	% TODO: investigate what is the best way to do this
	%	- chunking (below)
	%	- stream into collection
	%   - threading of mongolog_call
	%		- would be nice to just create a worker for each instantiation
	%         as it can be done outside. computable does the same.
	option(input_queue(InQueue), Options),
	option(goal_variables(Pattern), Options),
	message_queue_materialize(InQueue, Pattern),
	mongolog_call(Goal, Options).

%call_with(mongolog, Goal, Options) :-
%	option(input_queue(InQueue), Options),
%	option(goal_variables(Pattern), Options),
%	option(chunk_size(ChunkSize), Options, 10),
%	% retrieve next chunk
%	findnsols(ChunkSize,
%		InstantiatedPattern,
%		message_queue_materialize(InQueue, InstantiatedPattern),
%		NSolutions
%	),
%	% bake chunk into next goal to avoid outer choicepoints (only one per chunk)
%	% FIXME: member call below does not work for some reason
%	Goal0=','(member(Pattern, NSolutions), Goal),
%	mongolog_call(Goal0, Options).


		 /*******************************
		 *	    	OPTIONS		  	 	*
		 *******************************/

% name of default fact graph
:- dynamic default_graph/1.

default_graph(user).

%%
% Set the name of the graph where facts are asserted and retrieved
% if no other graph was specified.
%
set_default_graph(Graph) :-
	retractall(default_graph(_)),
	assertz(default_graph(Graph)).

%%
set_graph_option(Options, Options) :-
	option(graph(_), Options),
	!.
set_graph_option(Options, Merged) :-
	default_graph(DG),
	merge_options([graph(DG)], Options, Merged).


		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%%
% Term expansion for *ask* rules using the (?>) operator.
% The body is rewritten such that mng_ask is called instead
% with body as argument.
%
user:term_expansion(
		(?>(Head,Body)),
		(:-(HeadGlobal, lang_query:ask1(BodyGlobal, QScope, _FScope, [])))) :-
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,_Module,Term),
	current_scope(QScope),
	% add the rule to the DB backend
	mongolog_add_rule(Term, BodyGlobal).

%%
% Term expansion for *tell* rules using the (+>) operator.
% The rules are only asserted into mongo DB and expanded into
% empty list.
%
user:term_expansion(
		(+>(Head,Body)),
		[]) :-
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,_Module,Term),
	% rewrite functor
	% TODO: it would be nicer to generate a lot
	%        clauses for project/1.
	Term =.. [Functor|Args],
	atom_concat('project_',Functor,Functor0),
	Term0 =.. [Functor0|Args],
	% add the rule to the DB backend
	mongolog_add_rule(Term0, project(BodyGlobal)).

%%
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).

%%
% Term expansion for *tell-ask* rules using the (?+>) operator.
% These are basically clauses that can be used in both contexts.
%
% Consider for example following rule:
%
%     is_event(Entity) ?+>
%       has_type(Entity, dul:'Event').
%
% This is valid because, in this case, has_type/2 has
% clauses for ask and tell.
%
user:term_expansion((?+>(Head,Goal)), X1) :-
	user:term_expansion((?>(Head,Goal)),X1),
	user:term_expansion((+>(Head,Goal)),_X2).

