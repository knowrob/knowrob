:- module(lang_query,
    [ ask(t),        % +Statement
      ask(t,t,t),    % +Statement, +QScope, -FScope
      ask(t,t,t,t),  % +Statement, +QScope, -FScope, +Options
      tell(t),       % +Statement
      tell(t,t),     % +Statement, +Scope
      tell(t,t,t),   % +Statement, +Scope, +Options
      forget(t),     % +Statement
      forget(t,t),   % +Statement, +Scope
      forget(t,t,t)  % +Statement, +Scope, +Options
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
:- use_module('computable',
    [ computable_call/5, computable_split/4 ]).
:- use_module('mongolog/mongolog').

% fallback graph for tell/forget
:- dynamic default_graph/1.

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
	merge_options([
		scope(QScope),
		user_vars([['v_scope',FScope]|Fields])
	], Options, Options1),
	call_query(Goal, Options1).

%%
call_query(Goal, Options) :-
	% split query at first (chain of) computable predicate encountered
	% into subgoals left-of and sub-goals right-of computables
	computable_split(Goal, ComputationGoal, LeftGoal, RightGoal),
	!,
	% left-side of computables generates input that is streamed into
	% the computation process
	(	LeftGoal==[]
	->	GeneratorGoal=true % first sub-goal is a computable
	;	GeneratorGoal=mongolog_call(LeftGoal, Options)
	),
	% get list of variables that appear in RightGoal _and_ left of it.
	% these are the inputs when calling RightGoal
	shared_variables([ComputationGoal,LeftGoal], RightGoal, Pattern),
	% run the computation.
	% the instantiations are returned in chunks, and
	% computation_run creates choicepoints for different
	% chunks.
	computable_call(ComputationGoal, GeneratorGoal,
		Pattern, NSolutions, Options),
	% avoid choicepoints here, and bake instantiations
	% into the query
	RestGoal=','(member(Pattern, NSolutions), RightGoal),
	% finally do a recursive call
	call_query(RestGoal, Options).

call_query(Goal, Options) :-
	% Goal has no computables
	mongolog_call(Goal, Options).

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

%%
%
shared_variables(Term0, Term1, SharedVars) :-
	term_variables(Term0, Vars0),
	term_variables(Term1, Vars1),
	sort(Vars0, Sorted0),
	sort(Vars1, Sorted1),
	ord_intersection(Sorted0, Sorted1, SharedVars, _).
