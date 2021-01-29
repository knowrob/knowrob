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
:- use_module('compiler').

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
	once(query_ask(Statement, QScope, FScope, Options)).

ask(Statement, QScope, FScope, Options) :-
	%\+ ground(Statement),
	query_ask(Statement, QScope, FScope, Options).

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
	;	query_tell(Statement, Scope, Options0)
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
		(:-(HeadGlobal, query_ask(BodyGlobal, QScope, _FScope, [])))) :-
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,_Module,Term),
	current_scope(QScope),
	% add the rule to the DB backend
	query_assert((?>(Term, BodyGlobal))).

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
	% add the rule to the DB backend
	query_assert((+>(Term,BodyGlobal))).

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
user:term_expansion((?+>(Head,Goal)), [X1,X2]) :-
	user:term_expansion((?>(Head,Goal)),X1),
	user:term_expansion((+>(Head,Goal)),X2).


		 /*******************************
		 *      	  HELPER     		*
		 *******************************/

%%
set_graph_option(Options, Options) :-
	option(graph(_), Options),
	!.

set_graph_option(Options, Merged) :-
	default_graph(DG),
	merge_options([graph(DG)], Options, Merged).
