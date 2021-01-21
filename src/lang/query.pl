:- module(lang_query,
    [ ask(t),      % +Statement
      ask(t,t),    % +Statement, +Scope
      tell(t),     % +Statement
      tell(t,t),   % +Statement, +Scope
      forget(t),
      forget(t,t)
      %update(t),  % +Statement
      %update(t,t) % +Statement, +Scope
    ]).
/** <module> Main interface predicates for querying the knowledge base.

@author Daniel Beßler
@license BSD
*/

:- op(1100, xfx, user:(?>)).
:- op(1100, xfx, user:(+>)).
:- op(1100, xfx, user:(?+>)).

:- use_module(library('db/scope'),
    [ current_scope/1,
      universal_scope/1
    ]).
:- use_module(library('lang/scopes/temporal')).
:- use_module(library('compiler')).

% define some settings
:- setting(drop_graphs, list, [user],
		'List of named graphs that should initially by erased.').

%
:- dynamic default_graph/1.
%
default_graph(user).

% TODO search for where tripledb init is called
query_init :-
	% drop some graphs on start-up
	(	setting(mng_client:read_only, true)
	->	true
	;	(	setting(tripledb:drop_graphs,L),
			forall(member(X,L), drop_graph(X))
		)
 	).

%%
% Set the name of the graph where triples are asserted and retrieved
% if no other graph was specified.
%
set_default_graph(Graph) :-
	retractall(default_graph(_)),
	assertz(default_graph(Graph)).

%%
set_graph_option(Options,Options) :-
	option(graph(_),Options),!.

set_graph_option(Options,Merged) :-
	default_graph(DG),
	merge_options([graph(DG)],Options,Merged).

%%
% NOTE: SWI strip_module acts strange
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).


%% ask(+Statement) is nondet.
%
% Same as ask/2 with default scope to include
% only facts that hold now.
%
% @param Statement a statement term.
%
ask(Statement) :-
	current_scope(QScope),
	ask(Statement,[[],QScope]->_).

%% ask(+Statement,+Scope) is nondet.
%
% True if Statement term holds within the requested scope.
% Scope is a term `[Options,QueryScope]->FactScope` where QueryScope
% is the scope requested, and FactScope the actual scope
% of the statement being true.
% Statement can also be a list of statements.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
ask(Statements, QScope->FScope) :-
	is_list(Statements),
	!,
	comma_list(Goal, Statements),
	ask(Goal, Scope).

ask(Statement,Scope) :-
	% grounded statements have no variables
	% in this case we can limit to one solution here
	ground(Statement),
	!,
	once(ask1(Statement,Scope)).

ask(Statement,Scope) :-
	%\+ nonground(Statement),
	ask1(Statement,Scope).

%%
ask_1(Statement, [Options,QScope]->FScope) :-
	query_ask(Statement, QScope, FScope, Options).

%% tell(+Statement) is nondet.
%
% Same as tell/2 with universal scope.
%
% @param Statement a statement term.
%
tell(Statement) :-
	universal_scope(FScope),
	tell(Statement,[[],FScope]).

%% tell(+Statement,+Scope) is det.
%
% Tell the knowledge base that some statement is true.
% Scope is a term `[Options,FactScope]` where FactScope
% the scope of the statement being true.
% Statement can also be a list of statements.
%
% tell/2 is a multifile predicate. Meaning that clauses may be
% decalared in multiple files.
% Specifically, declaring a rule using the tell operator `+>`,
% or the ask-tell operator `?+>` will generate a clause of the tell rule.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
tell(Statements, Scope) :-
	is_list(Statements),
	!,
	comma_list(Goal, Statements),
	tell(Goal, Scope).

tell(Statement, [Options,Scope]) :-
	% TODO
%	(	setting(mng_client:read_only, true)
%  	-> print_message(warning, 'Tried to write despite read only access')
%  	;	itripledb_tell(S,P,O,Scope,Options0)
%  	),
	query_tell(Statement, FScope, Options).

%%
forget(Statement) :-
	wildcard_scope(Scope),
	forget(Statement, [[],Scope]).

forget(Statement, [Opt,Scope]) :-
	set_graph_option(Opt, Opt0),
	(	setting(mng_client:read_only, true)
	->	print_message(warning, 'Tried to delete despite read only access')
	;	forget_(Statement, [Opt0,Scope])
	).

%%
drop_graph(Name) :-
	wildcard_scope(QScope),
	forget(triple(_,_,_), [[graph(=(Name))],QScope]).

%% update(+Statement) is nondet.
%
% Same as tell/1 but replaces existing overlapping values.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
%update(Statement) :-
%	universal_scope(FScope),
%	update(Statement,[[],FScope]).

%% update(+Statement,+Scope) is nondet.
%
% Same as tell/2 but replaces existing overlapping values.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
%update(Statement,Scope0) :-
%	context_update_(Scope0,[options([functional])],Scope1),
%	tell(Statement,Scope1).

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
		(:-(Head,ask_query(Body, QScope, _FScope, [])))) :-
	strip_module_(Head,_Module,Term),
	current_scope(QScope),
	query_assert((?>(Term,Body))).

%%
% Term expansion for *tell* rules using the (+>) operator.
% The rules are only asserted into mongo DB and expanded into
% empty list.
%
user:term_expansion(
		(+>(Head,Body)),
		[]) :-
	strip_module_(Head,_Module,Term),
	query_assert((+>(Term,Body))).

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

