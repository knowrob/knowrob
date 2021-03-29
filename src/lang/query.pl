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
      kb_add_rule(t,t),
	  kb_drop_rule(t),
	  kb_expand(t,-),
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

% Stores list of terminal terms for each clause. 
:- dynamic kb_rule/3.
% optionally implemented by query commands.
:- multifile step_expand/2.
% interface implemented by query backends
:- multifile is_callable_with/2.
:- multifile call_with/3.

% create a thread pool for query processing
:- worker_pool_create('lang_query:queries').
:- current_prolog_flag(cpu_count, NumCPUs),
   worker_pool_create('lang_query:backends', [max_size(NumCPUs)]).

% the named thread pool used for query processing
query_thread_pool('lang_query:queries').
backend_thread_pool('lang_query:backends').

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
	kb_expand(Goal, Expanded),
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
	query_thread_pool(Pool),
	forall(
		% iterate over different backends
		member([Backend,InQueue], Channels),
		% run call_with/3 in a thread for each backend
		worker_pool_start_work(Pool, InQueue,
			lang_query:call_with(
				Backend, Goal, Pattern,
				OutQueue, InQueue, Options
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
	query_thread_pool(QueryPool),
	backend_thread_pool(BackendPool),
	worker_pool_stop_work(QueryPool,OutQueue),
	worker_pool_stop_work(BackendPool,OutQueue),
	forall(
		member([_,InQueue], Channels),
		worker_pool_stop_work(QueryPool,InQueue)
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
	query_thread_pool(Pool),
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
	is_mongolog_predicate(Goal).

% call Goal in Backend and send result on OutQueue
call_with(Backend, Goal, Pattern, OutQueue, InQueue, Options) :-
	% TODO: in some cases it could be beneficial to do chunking
	%       then run worker threads for each chunk
	%
	backend_thread_pool(BackendPool),
	% pass each instantiation to a worker thread
	forall(
		message_queue_materialize(InQueue, Pattern),
		worker_pool_start_work(BackendPool, OutQueue,
			lang_query:call_with0(Backend, Goal, Pattern, OutQueue, Options)
		)
	),
	% need to wait until all backend workers have completed
	worker_pool_wait(BackendPool, OutQueue),
	% then we send eos message
	thread_send_message(OutQueue, end_of_stream).

%
call_with0(Backend, Goal, Pattern, OutQueue, Options) :-
	% pass any error to output queue consumer
	catch(
		call_with1(Backend, Goal, Pattern, OutQueue, Options),
		Error,
		thread_send_message(OutQueue, error(Error))
	).

%
call_with1(Backend, Goal, Pattern, OutQueue, Options) :-
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
call_with(mongolog, Goal, Options) :- mongolog_call(Goal, Options).


		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%% kb_add_rule(+Head, +Body) is semidet.
%
% Register a rule that translates into an aggregation pipeline.
% Any non-terminal predicate in Body must have a previously asserted
% mongolog rule it can expand into.
% After being asserted, the Head predicate can be referred to in
% calls of mongolog_call/1.
%
% @param Head The head of a rule.
% @param Body The body of a rule.
%
kb_add_rule(Head, Body) :-
	%% get the functor of the predicate
	Head =.. [Functor|Args],
	%% expand goals into terminal symbols
	(	kb_expand(Body, Expanded) -> true
	;	log_error_and_fail(lang(assertion_failed(Body), Functor))
	),
	%% handle instantiated arguments
	% FIXME: BUG: this might create problems in cases expanded arg is nongrounded term.
	%             then to instantiate the args implicit instantiation might be needed.
	%             A solution could be adding a =/2 call _at the end_ to avoid the need
	%             for implicit instantiation.
	(	expand_arguments(Args, ExpandedArgs, ArgsGoal)
	->	(	(	is_list(Expanded)
			->	Expanded0=[ArgsGoal|Expanded]
			;	Expanded0=[ArgsGoal,Expanded]
			),
			Args0=ExpandedArgs
		)
	;	(	Expanded0=Expanded,
			Args0=Args
		)
	),
	%% store expanded query
	assertz(kb_rule(Functor, Args0, Expanded0)).


%% kb_drop_rule(+Head) is semidet.
%
% Drop a previously added `mongolog` rule.
% That is, erase its database record such that it can
% not be referred to anymore in rules added after removal.
%
% @param Term A mongolog rule.
%
kb_drop_rule(Head) :-
	compound(Head),
	Head =.. [Functor|_],
	retractall(kb_rule(Functor, _, _)).

%%
expand_arguments(Args, Expanded, pragma(=(Values,Vars))) :-
	expand_arguments1(Args, Expanded, Pairs),
	Pairs \= [],
	pairs_keys_values(Pairs, Values, Vars).


expand_arguments1([], [], []) :- !.
expand_arguments1([X|Xs], [X|Ys], Zs) :-
	var(X),!,
	expand_arguments1(Xs, Ys, Zs).
expand_arguments1([X|Xs], [Y|Ys], [X-Y|Zs]) :-
	expand_arguments1(Xs, Ys, Zs).


		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%% kb_expand(+Term, -Expanded) is det.
%
% Translate a goal into a sequence of terminal commands.
% Terminal commands are the core predicates supported in queries
% such as arithmetic and comparison predicates.
% Rules, on the other hand, are "flattened" during term expansion,
% and translated to a sequence of these terminal commands.
%
% @param Term A compound term, or a list of terms.
% @param Expanded Sequence of terminal commands
%
kb_expand(Goal, Goal) :-
	% goals maybe not known during expansion, i.e. in case of
	% higher-level predicates receiving a goal as an argument.
	% these var goals need to be expanded compile-time
	% (call-time is not possible)
	var(Goal), !.

kb_expand(Goal, Expanded) :-
	% NOTE: do not use is_list/1 here, it cannot handle list that have not
	%       been completely resolved as in `[a|_]`.
	%       Here we check just the head of the list.
	\+ has_list_head(Goal), !,
	comma_list(Goal, Terms),
	kb_expand(Terms, Expanded).

kb_expand(Terms, Expanded) :-
	has_list_head(Terms), !,
	catch(
		expand_term_0(Terms, Expanded0),
		Exc,
		log_error_and_fail(lang(Exc, Terms))
	),
	comma_list(Buf,Expanded0),
	comma_list(Buf,Expanded1),
	% Handle cut after term expansion.
	% It is important that this is done _after_ expansion because
	% the cut within the call would yield an infinite recursion
	% otherwhise.
	% TODO: it is not so nice doing it here. would be better if it could be
	%       done in control.pl where cut operator is implemented but current
	%       interfaces don't allow to do the following operation in control.pl.
	%		(without special handling of ',' it could be done, I think)
	expand_cut(Expanded1, Expanded2),
	%%
	(	Expanded2=[One] -> Expanded=One
	;	Expanded=Expanded2
	).

%%
expand_term_0([], []) :- !.
expand_term_0([X|Xs], [X_expanded|Xs_expanded]) :-
	once(expand_term_1(X, X_expanded)),
	% could be that expand-time the list is not fully resolved
	(	var(Xs) -> Xs_expanded=Xs
	;	expand_term_0(Xs, Xs_expanded)
	).

expand_term_1(Goal, Expanded) :-
	% TODO: seems nested terms sometimes not properly flattened, how does it happen?
	is_list(Goal),!,
	expand_term_0(Goal, Expanded).

expand_term_1(Goal, Expanded) :-
	once(is_callable_with(_,Goal)),
	%Goal =.. [Functor|_Args],
	%step_command(Functor),
	% allow the goal to recursively expand
	(	step_expand(Goal, Expanded) -> true
	;	Expanded = Goal
	).

expand_term_1(Goal, Expanded) :-
	% find all asserted rules matching the functor and args
	Goal =.. [Functor|Args],
	% NOTE: do not use findall here because findall would not preserve
	%       variables in Terminals
	% NOTE: Args in query only contain vars, for instantiated vars in rule
	%       heads, pragma/1 calls are generated in Terminals (i.e. the body of the rule).
	(	bagof(Terminals,
			kb_rule(Functor, Args, Terminals),
			TerminalsList)
	->	true
	% handle the case that a predicate is referred to that wasn't
	% asserted before
	;	throw(expansion_failed(Goal))
	),
	% wrap different clauses into ';'
	semicolon_list(Goal0, TerminalsList),
	kb_expand(Goal0, Expanded).

%%
% Each conjunction with cut operator [X0,...,Xn,!|_]
% is rewritten as [limit(1,[X0,....,Xn])|_].
%
expand_cut([],[]) :- !.
expand_cut(Terms,Expanded) :-
	take_until_cut(Terms, Taken, Remaining),
	% no cut if Remaining=[]
	(	Remaining=[] -> Expanded=Terms
	% else the first element in Remaining must be a cut
	% that needs to be applied to goals in Taken
	;	(	Remaining=[!|WithoutCut],
			expand_cut(WithoutCut, Remaining_Expanded),
			Expanded=[limit(1,Taken)|Remaining_Expanded]
		)
	).

%
step_expand(ask(Goal), ask(Expanded)) :-
	kb_expand(Goal, Expanded).

% split list at cut operator.
take_until_cut([],[],[]) :- !.
take_until_cut(['!'|Xs],[],['!'|Xs]) :- !.
take_until_cut([X|Xs],[X|Ys],Remaining) :-
	take_until_cut(Xs,Ys,Remaining).

% 
has_list_head([]) :- !.
has_list_head([_|_]).


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
	kb_add_rule(Term, BodyGlobal).

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
	kb_add_rule(Term0, project(BodyGlobal)).

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

%%
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).


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

