:- module(blackboard,
    [ kb_call(t),             % +Goal
      kb_call(t,t,t),         % +Goal, +QScope, -FScope
      kb_call(t,t,t,t),       % +Goal, +QScope, -FScope, +Options
      kb_project(t),          % +Goal
      kb_project(t,t),        % +Goal, +Scope
      kb_project(t,t,t),      % +Goal, +Scope, +Options
      kb_unproject(t),        % +Goal
      kb_unproject(t,t),      % +Goal, +Scope
      kb_unproject(t,t,t)     % +Goal, +Scope, +Options
    ]).
/** <module> Query evaluation via a blackboard.

@author Daniel Beßler
@license BSD
*/

:- use_module(library(settings)).
:- use_module(library('semweb/rdf_db'),
	[ rdf_global_term/2 ]).
:- use_module(library('scope'),
    [ current_scope/1, universal_scope/1 ]).

%% TODO
:- setting(mng_client:collection_names, list, [triples, tf, annotations, inferred],
		'List of collections that will be imported/exported with remember/memorize.').

%%
% Assert the collection names to be used by remember/memorize
%
auto_collection_names :-
	setting(mng_client:collection_names, L),
	forall(member(X,L), assertz(collection_name(X))).

:- ignore(auto_collection_names).

/*
%%
mng_import(Dir) :-
	forall(
		(	collection_name(Name),
			mng_get_db(DB, Collection, Name)
		),
		(	path_concat(Dir, Collection, Dir0),
			mng_restore(DB, Dir0)
		)
	).


%%
mng_export(Dir) :-
	forall(
		(	collection_name(Name),
			mng_get_db(DB, Collection, Name)
		),
		(	path_concat(Dir, Collection, Dir0),
			mng_dump_collection(DB, Collection, Dir0)
		)
	).
*/

%% kb_call(+Statement) is nondet.
%
% Same as kb_call/3 with default scope to include
% only facts that hold now.
%
% @param Statement a statement term.
%
kb_call(Statement) :-
	current_scope(QScope),
	kb_call(Statement, QScope, _, []).

%% kb_call(+Statement, +QScope, -FScope) is nondet.
%
% Same as kb_call/4 with empty options list.
%
% @param Statement a statement term.
%
kb_call(Statement, QScope, FScope) :-
	kb_call(Statement, QScope, FScope, []).

%% kb_call(+Statement, +QScope, -FScope, +Options) is nondet.
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
kb_call(Statements, QScope, FScope, Options) :-
	is_list(Statements),
	!,
	comma_list(Goal, Statements),
	kb_call(Goal, QScope, FScope, Options).

kb_call(Statement, QScope, FScope, Options) :-
	% grounded statements have no variables
	% in this case we can limit to one solution here
	ground(Statement),
	!,
	once(kb_call0(Statement, QScope, FScope, Options)).

kb_call(Statement, QScope, FScope, Options) :-
	%\+ ground(Statement),
	kb_call0(Statement, QScope, FScope, Options).

%%
kb_call0(Goal, QScope, FScope, Options) :-
	option(fields(Fields), Options, []),
	% add all toplevel variables to context
	% FIXME: this seems to be mongolog related, move it there!
	term_keys_variables_(Goal, GlobalVars),
	merge_options(
		[ scope(QScope),
		  user_vars([['v_scope',FScope]|Fields]),
		  global_vars(GlobalVars)
		],
		Options, Options1),
	kb_call1(Goal, Options1).

%%
/*
kb_call1(SubGoals, Options) :-
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
*/

%
term_keys_variables_(Goal, GoalVars) :-
	term_variables(Goal, Vars),
	term_keys_variables_1(Vars, GoalVars).
term_keys_variables_1([], []) :- !.
term_keys_variables_1([X|Xs], [[Key,X]|Ys]) :-
	term_to_atom(X,Atom),
	atom_concat('v',Atom,Key),
	term_keys_variables_1(Xs, Ys).

%% kb_project(+Statement) is nondet.
%
% Same as kb_project/2 with universal scope.
%
% @param Statement a statement term.
%
kb_project(Statement) :-
	universal_scope(Scope),
	kb_project(Statement, Scope, []).

%% kb_project(+Statement, +Scope) is nondet.
%
% Same as kb_project/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
kb_project(Statement, Scope) :-
	kb_project(Statement, Scope, []).

%% kb_project(+Statement, +Scope, +Options) is semidet.
%
% Assert that some statement is true.
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
kb_project(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	comma_list(Statement, Statements),
	kb_project(Statement, Scope, Options).

kb_project(Statement, Scope, Options) :-
	% ensure there is a graph option
	set_graph_option(Options, Options0),
	/*
	% compile and call statement
	(	setting(mng_client:read_only, true)
	->	log_warning(db(read_only(projection)))
	;	mongolog_call(project(Statement), [scope(Scope)|Options0])
	).
	*/
	kb_project1(Statement, [scope(Scope)|Options0]).


%% kb_unproject(+Statement) is nondet.
%
% Same as kb_unproject/2 with universal scope.
%
% @param Statement a statement term.
%
kb_unproject(Statement) :-
	wildcard_scope(Scope),
	kb_unproject(Statement, Scope, []).

%% kb_unproject(+Statement, +Scope) is nondet.
%
% Same as kb_unproject/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
kb_unproject(Statement, Scope) :-
	kb_unproject(Statement, Scope, []).

%% kb_unproject(+Statement, +Scope, +Options) is semidet.
%
% Unproject that some statement is true.
% Statement must be a term triple/3. 
% It can also be a list of such terms.
% Scope is the scope of the statement to unproject. Options include:
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

kb_unproject(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	forall(
		member(Statement, Statements),
		kb_unproject(Statement, Scope, Options)
	).
