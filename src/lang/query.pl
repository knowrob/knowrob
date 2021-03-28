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

% fallback graph for tell/forget
:- dynamic default_graph/1.

:- multifile is_callable_with/2.
:- multifile call_with/3.

% create a thread pool for query processing
:- worker_pool_create('lang_query').

% the named thread pool used for query processing
thread_pool('lang_query').

%%
% Set the name of the graph where triples are asserted and retrieved
% if no other graph was specified.
%
set_default_graph(Graph) :-
	retractall(default_graph(_)),
	assertz(default_graph(Graph)).

:- set_default_graph(user).

%%
% NOTE: SWI strip_module acts strange
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).

%% ask(+Statement, +QScope, -FScope, +Options) is nondet.
%
% True if Statement term holds within the requested scope (QScope).
% Statement can also be a list of statements.
% FactScope the actual scope of the statement being true that overlaps
% with the requested scope.
% The list of options is passed to the compiler.
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
	once(call_query(Statement, QScope, FScope, Options)).

ask(Statement, QScope, FScope, Options) :-
	%\+ ground(Statement),
	call_query(Statement, QScope, FScope, Options).

%% ask(+Statement, +QScope, -FScope) is nondet.
%
% Same as ask/4 with empty options list.
%
% @param Statement a statement term.
%
ask(Statement, QScope, FScope) :-
	ask(Statement, QScope, FScope, []).

%% ask(+Statement) is nondet.
%
% Same as ask/2 with default scope to include
% only facts that hold now.
%
% @param Statement a statement term.
%
ask(Statement) :-
	current_scope(QScope),
	ask(Statement, QScope, _, []).

%%
call_query(Goal, QScope, FScope, Options) :-
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
	call_query(Flattened, Options1).

%%
call_query(SubGoals, Options) :-
	% create a list of step(SubGoal, OutQueue, Backends) terms
	maplist([SubGoal,Step]>>
		query_step(SubGoal,Step),
		SubGoals, Steps),
	% combine steps if possible
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


%% tell(+Statement, +Scope, +Options) is semidet.
%
% Tell the knowledge base that some statement is true.
% Scope is the scope of the statement being true.
% Statement can also be a list of statements.
% The list of options is passed to the compiler.
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

%% tell(+Statement, +Scope) is nondet.
%
% Same as tell/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
tell(Statement, Scope) :-
	tell(Statement, Scope, []).

%% tell(+Statement) is nondet.
%
% Same as tell/2 with universal scope.
%
% @param Statement a statement term.
%
tell(Statement) :-
	universal_scope(Scope),
	tell(Statement, Scope, []).

%% forget(+Statement, +Scope, +Options) is semidet.
%
% Forget that some statement is true.
% Statement must be a term triple/3. 
% It can also be a list of such terms.
% Scope is the scope of the statement to forget.
% The list of options is passed to the compiler.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
% @param Options list of options.
%
forget(_, _, _) :-
	setting(mng_client:read_only, true),
	!,
	log_warning(db(read_only(forget))).

forget(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	forall(
		member(Statement, Statements),
		forget(Statement, Scope, Options)
	).

% TODO: support other language terms?
% FIXME: need to propagate deletion for rdf:type etc.
forget(triple(S,P,O), Scope, Options) :-
	% ensure there is a graph option
	set_graph_option(Options, Options0),
	% append scope to options
	merge_options([scope(Scope)], Options0, Options1),
	% get the query document
	mng_triple_doc(triple(S,P,O), Doc, Options1),
	% run a remove query
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, Doc).

%% forget(+Statement, +Scope) is nondet.
%
% Same as forget/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
forget(Statement, Scope) :-
	forget(Statement, Scope, []).

%% forget(+Statement) is nondet.
%
% Same as forget/2 with universal scope.
%
% @param Statement a statement term.
%
forget(Statement) :-
	wildcard_scope(Scope),
	forget(Statement, Scope, []).


%%
set_graph_option(Options, Options) :-
	option(graph(_), Options),
	!.
set_graph_option(Options, Merged) :-
	default_graph(DG),
	merge_options([graph(DG)], Options, Merged).


		 /*******************************
		 *	    query pipelines		   	*
		 *******************************/


% start processing all steps of a pipeline
start_pipeline([FirstStep|Rest], Pattern, Options, FinalStep) :-
	% create step message queues
	create_step_queues(FirstStep),
	% send uninstantiated pattern as starting point
	% for the first step.
	forall(
		step_input(FirstStep,InQueue),
		(	thread_send_message(InQueue,Pattern),
			thread_send_message(InQueue,end_of_stream)
		)
	),
	start_pipeline1([FirstStep|Rest], Pattern, Options, FinalStep).

%
start_pipeline1([Step|Rest], Pattern, Options, FinalStep) :-
	% create step message queues
	create_step_queues(Step),
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
	% TODO: use pool of message queues to avoid creating them
	%       each time a goal is called?
	findall([X,_], is_callable_with(X,Goal), Channels),
	Channels \== [].


% ensure message queues associated to a step are instantiated
create_step_queues(step(_, OutQueue, Channels)) :-
	create_queue_(OutQueue),
	create_step_queues1(Channels).

create_step_queues1([]) :- !.
create_step_queues1([[_,InQueue]|Rest]) :-
	create_queue_(InQueue),
	create_step_queues1(Rest).

%
create_queue_(Queue) :- nonvar(Queue),!.
create_queue_(Queue) :-
	% TODO: what would be a good maximum number of
	%       messages in the queue? 
	%message_queue_create(Queue, [max_size(100)]),
	message_queue_create(Queue).

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

%
connect_steps(LastStep, ThisStep) :-
	step_output(LastStep, OutQueue),
	catch(
		message_queue_materialize(OutQueue, Msg),
		Error,
		Msg=error(Error) % pass exception to next step
	),
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
		 *	    querying backends   	*
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


% stream results of call_with/3 into OutQueue
call_with(Backend, Goal, OutQueue, Options) :-
	option(goal_variables(Pattern), Options),
	catch(
		(	% call goal in backend
			call_with(Backend, Goal, Options),
			% publish result via OutQueue
			thread_send_message(OutQueue, Pattern)
		),
		Error,
		% pass error to output queue consumer
		thread_send_message(OutQueue, error(Error))
	).

call_with(_Backend, _Goal, OutQueue, _Options) :-
	% send end_of_stream message needed
	% to terminate worker waiting for next result.
	thread_send_message(OutQueue, end_of_stream).


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
	option(input_queue(InQueue), Options),
	option(goal_variables(Pattern), Options),
	% TODO: think about chunking strategies
	%	- query has a limited size
%	option(chunk_size(ChunkSize), Options, 100),
	% retrieve next chunk
%	findnsols(ChunkSize,
%		InstantiatedPattern,
%		message_queue_materialize(InQueue, InstantiatedPattern),
%		NSolutions
%	),
	% bake chunk into next goal to avoid outer choicepoints
	% (only one per chunk)
	% TODO: better stream into a collection, then draw instantiations
	%       from that colection with a stream query?
	%       then there would be no need to bake chunks into the query.
%	Goal0=','(member(Pattern, NSolutions), Goal),
%	mongolog_call(Goal0, Options),
	message_queue_materialize(InQueue, Pattern),
	mongolog_call(Goal, Options).


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
		(:-(HeadGlobal, lang_query:call_query(BodyGlobal, QScope, _FScope, [])))) :-
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

