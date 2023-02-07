:- module(blackboard,
    [ current_reasoner_module/1,      % -ReasonerModule
      set_current_reasoner_module/1,  % +ReasonerModule
      reasoner_defined_predicate/2,   % ?PredicateIndicator, ?PredicateType
      reasoner_setting/2,             % +Name, ?Value
      reasoner_setting/4,             % +Name, +Type, +Default, +Comment
      reasoner_set_setting/3,         % +ResonerModule, +Name, +Value
      reasoner_rdf_init/1,            % +ResonerModule
      reasoner_call/2
/*    kb_call(t),             % +Goal
      kb_call(t,t,t),         % +Goal, +QScope, -FScope
      kb_call(t,t,t,t),       % +Goal, +QScope, -FScope, +Options
      kb_project(t),          % +Goal
      kb_project(t,t),        % +Goal, +Scope
      kb_project(t,t,t),      % +Goal, +Scope, +Options
      kb_unproject(t),        % +Goal
      kb_unproject(t,t),      % +Goal, +Scope
      kb_unproject(t,t,t)     % +Goal, +Scope, +Options
      */
    ]).

/** <module> Query evaluation via a blackboard.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('settings'), [ setting/2 ]).
:- use_module(library('logging')).

%% current_reasoner_module(?Reasoner) is semidet.
%
% Unifies Reasoner with the reasoner currently active in the current thread.
% Note that the current implementation uses global variables and requires
% that the global variable is set at the beginning of each query using set_current_reasoner_module/1.
%
current_reasoner_module(Reasoner) :-
    nb_current(reasoner_module, Reasoner),
    !.
current_reasoner_module(user).

%% set_current_reasoner_module(+Reasoner) is det.
%
% Sets the reasoner active in the curent thread.
% Prolog declarations may be loaded into a module named after
% the reasoner.
%
set_current_reasoner_module(Reasoner) :-
    b_setval(reasoner_module, Reasoner).

%% reasoner_setting(:Name, ?Value) is nondet.
%
% True when Name is a currently defined setting with Value.
% The setting must be defined in the currently used reasoner module.
% It is assumed that the toplevel module in the import hierarchy
% is the reasoner instance module.
%
reasoner_setting(Name, Value) :-
    current_reasoner_module(ReasonerModule),
    reasoner_setting_(ReasonerModule, Name, Value).

reasoner_setting_(ReasonerModule, Name, Value) :-
    catch(
        ReasonerModule:call(current_setting(Name,Value)),
        error(existence_error(procedure, _), _),
        fail
    ),!.

reasoner_setting_(_ReasonerModule, Name, Default) :-
    user:defined_reasoner_setting(Name,_,Default,_),!.

%% reasoner_setting(+Name, +Type, +Default, +Comment) is det.
%
% Define a setting. Name denotes the name of the setting, Type its type.
% Multiple declarations are silently ignored.
%
reasoner_setting(Name, _Type, _Default, _Comment) :-
    % already defined? ignore
    user:defined_reasoner_setting(Name,_,_,_),
    !.

reasoner_setting(Name, Type, Default, Comment) :-
    user:assertz(defined_reasoner_setting(Name, Type, Default, Comment)).

%%
reasoner_set_setting(ResonerModule, Name, ValueString) :-
    setting_value1(Name, ValueString, Value),
    ResonerModule:transaction((
        retractall(current_setting(Name,_)),
        assertz(current_setting(Name,Value))
    )).

%%
setting_value1(Name, ValueString, Value) :-
    user:defined_reasoner_setting(Name,Type,_,_),
    setting_value2(Type, ValueString, Value), !.

setting_value1(_Name, ValueString, ValueString) :- !.

setting_value2(list, Atom, Term)    :- term_to_atom(Term, Atom).
setting_value2(number, Atom, Term)  :- atom_number(Atom, Term).
setting_value2(float, Atom, Term)   :- atom_number(Atom, Term).
setting_value2(double, Atom, Term)  :- atom_number(Atom, Term).
setting_value2(integer, Atom, Term) :- atom_number(Atom, Term).
setting_value2(_, Atom, Atom).

%% reasoner_rdf_init(+Reasoner) is det.
%
% RDF facts are not stored as regular Prolog facts,
% and accessed through dedicated predicates, e.g. `rdf_has/2`.
% Each reasoner defines its own version of a set of such predicates:
%
%   - triple/3
%   - instance_of/2
%   - subclass_of/2
%   - subproperty_of/2
%
% This call adds definitions for the mentioned predicates in the
% module of the reasoner. The added definitions are just wrapper
% around the predicates accessing the database.
%
reasoner_rdf_init(Reasoner) :-
    % make sure to initialize only once
    Reasoner:current_predicate(semweb_initialized/0),
    !.

reasoner_rdf_init(Reasoner) :-
    % Prolog reasoner always use the built-in rdf library of Prolog.
    % semweb provides wrappers around these.
    Reasoner:assert(semweb_initialized),
    Reasoner:assert(':-'(triple(S,P,O),            semweb:sw_triple(S,P,O))),
    Reasoner:assert(':-'(instance_of(S,Cls),       semweb:sw_instance_of(S,Cls))),
    Reasoner:assert(':-'(instance_of_expr(S,Cls),  semweb:sw_instance_of_expr(S,Cls))),
    Reasoner:assert(':-'(subclass_of(Sub,Sup),     semweb:sw_subclass_of(Sub,Sup))),
    Reasoner:assert(':-'(subproperty_of(Sub,Sup),  semweb:sw_subproperty_of(Sub,Sup))),
    %%
    Reasoner:assert(':-'(triple(S,P,O,Ctx),        semweb:sw_triple(S,P,O,Ctx))),
    Reasoner:assert(':-'(instance_of(S,Cls,Ctx),   semweb:sw_instance_of(S,Cls,Ctx))).

%% reasoner_defined_predicate(?Indicator, ?PredicateType) is nondet.
%
% True if Name/Arity is the indicator of a predicate defined by the
% current reasoner, and which has the type PredicateType where
% PredicateType can be one of *built_in* or *relation*.
%
reasoner_defined_predicate(Name/Arity, PredicateType) :-
    nonvar(Name),!,
    functor(Head, Name, Arity),
    current_reasoner_module(Reasoner),
    reasoner_defined_predicate_1(Reasoner, Head, PredicateType).

reasoner_defined_predicate(Name/Arity, PredicateType) :-
    %var(Name),!,
    reasoner_defined_predicate_2(Name, Arity, PredicateType).

%
reasoner_defined_predicate_1(_, Head, built_in) :-
    system:predicate_property(Head, visible), !.

reasoner_defined_predicate_1(_, Head, built_in) :-
    user:predicate_property(Head, visible), !.

% TODO: in case IDB vs. EDB needs to be distinguished:
%reasoner_defined_predicate_1(Reasoner, Head, relation(idb)) :-
%    Reasoner:predicate_property(Head, number_of_rules(NumRules)),
%    NumRules > 0, !.

reasoner_defined_predicate_1(Reasoner, Head, relation) :-
    \+ user:predicate_property(Head, defined),
    Reasoner:predicate_property(Head, visible), !.

%
reasoner_defined_predicate_2(Name, Arity, built_in) :-
    system:predicate_property(Head, defined),
    functor(Head, Name, Arity),
    \+ sub_atom(Name, 0, _, _, $).

reasoner_defined_predicate_2(Name, Arity, built_in) :-
    user:predicate_property(Head, defined),
    \+ system:predicate_property(Head, defined),
    functor(Head, Name, Arity).

reasoner_defined_predicate_2(Name, Arity, relation) :-
    current_reasoner_module(Reasoner),
    Reasoner:predicate_property(Head, defined),
    \+ user:predicate_property(Head, defined),
    functor(Head, Name, Arity).

%% reasoner_call(+Goal, +QueryContext) is nondet.
%
% option(instantiations(List)): List is a list of predicate instantations
%
reasoner_call(Goal, QueryContext) :-
    % note: PrologReasoner does not support scoped queries
    current_reasoner_module(Reasoner),
    % record instantiated predicates into a list if requested
    (   option(predicates(Instantiations), QueryContext)
    ->  expand_instantiations(Goal, Expanded, Instantiations)
    ;   Expanded = Goal
    ),
    Reasoner:call(Expanded).

%%
expand_instantiations(Goal, Expanded, Instantiations) :-
    % use difference lists for O(1) append
	expand_instantiations1(Goal, Expanded, Instantiations-[]).

expand_instantiations1((Goal0, Goal1), (Expanded0, Expanded1), Xs-Zs) :-
    % conjunctive query
	expand_instantiations1(Goal0,Expanded0,Xs-Ys),
	expand_instantiations1(Goal1,Expanded1,Ys-Zs), !.

expand_instantiations1((Goal0 ; Goal1), (Expanded0 ; Expanded1), Xs-Ys) :-
    % disjunctive query
	expand_instantiations1(Goal0,Expanded0,Xs-Ys),
	expand_instantiations1(Goal1,Expanded1,Xs-Ys), !.
expand_instantiations1((X0 | X1), Expanded, Xs-Ys) :-
    expand_instantiations1((X0 ; X1), Expanded, Xs-Ys).

expand_instantiations1(ignore(Goal), once(Expanded ; Xs=Ys), Xs-Ys) :-
    % special handling for ignore/1 case where there is no solution
    expand_instantiations1(Goal, Expanded, Xs-Ys), !.

% explicitely not expand some of the meta predicates arguments
expand_instantiations1((\+ Goal), (\+ Goal, Xs=Ys),   Xs-Ys) :- !.
expand_instantiations1(not(Goal), (not(Goal), Xs=Ys), Xs-Ys) :- !.
expand_instantiations1(forall(Cond,Action),
    (forall(Cond,Action), Xs=Ys), Xs-Ys) :- !.

expand_instantiations1(Head, Expanded, Xs-Ys) :-
    % meta predicates like `call/1`, `once/1`, `->\2` that receive a goal as an argument,
    % and where the goal argument can be recursively expanded.
    % TODO: findall-family of predicates cannot change the list of instantiations in their goal argument.
    %   in fact, each predicate would need to appear in the pattern of the findall such that it is
    %   recorded. Hence, findall-family of predicates need in addition a rewriting of the pattern
    %   argument, adding the list of instantiations. but the meaning would not be clear. and also
    %   uses of the list would need to be rewritten. seems difficult.
    %   currently instantations made during the evaluation of findall predicates
    %   are not included in the Instantiations list.
	user:predicate_property(Head, defined),
	user:predicate_property(Head, meta_predicate(MetaSpecifier)), !,
	expand_meta_instantiations(Head, MetaSpecifier, Expanded, Xs-Ys).

expand_instantiations1(Head, (Head, Xs=Ys), Xs-Ys) :-
    % predicates defined in the "user" module are seen as builtins
	user:predicate_property(Head, defined), !.

% if above clauses fail, assume it's a user-defined relation without further checking.
% make sure the predicate is added to the list of instantiated predicates.
expand_instantiations1(Head, (Head, Xs=[Head|Ys]), Xs-Ys).

%%
expand_meta_instantiations(Head, MetaSpecifier, Expanded, Xs-Ys) :-
	Head =.. [HeadFunctor|Args],
	MetaSpecifier =.. [_MetaFunctor|Modes],
	expand_meta_instantiations1(Args, Modes, ExpandedArgs, Xs-Ys),
	Expanded =..  [HeadFunctor|ExpandedArgs].

%%
expand_meta_instantiations1([], [], [], Xs-Xs) :- !.

expand_meta_instantiations1([ArgN|Args], [ModeN|Modes], [Next|Rest], Xs-Zs) :-
	number(ModeN),!, % higher-order argument
	expand_instantiations1(ArgN,Next,Xs-Ys),
    expand_meta_instantiations1(Args,Modes,Rest,Ys-Zs).

expand_meta_instantiations1([ArgN|Args], [_|Modes], [ArgN|Rest], Xs-Ys) :-
    expand_meta_instantiations1(Args,Modes,Rest,Xs-Ys).

%%
% Assert the collection names to be used by remember/memorize
%
/*
auto_collection_names :-
	setting(mng_client:collection_names, L),
	forall(member(X,L), assertz(collection_name(X))).

:- ignore(auto_collection_names).
*/

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

% TODO: allow prolog rules to issue queries for other reasoner
%% kb_call(+Statement) is nondet.
%
% Same as kb_call/3 with default scope to include
% only facts that hold now.
%
% @param Statement a statement term.
%

%% kb_call(+Statement, +QScope, -FScope) is nondet.
%
% Same as kb_call/4 with empty options list.
%
% @param Statement a statement term.
%

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

%% kb_project(+Statement) is nondet.
%
% Same as kb_project/2 with universal scope.
%
% @param Statement a statement term.
%

%% kb_project(+Statement, +Scope) is nondet.
%
% Same as kb_project/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%

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


%% kb_unproject(+Statement) is nondet.
%
% Same as kb_unproject/2 with universal scope.
%
% @param Statement a statement term.
%

%% kb_unproject(+Statement, +Scope) is nondet.
%
% Same as kb_unproject/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%

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
