:- module(reasoner,
    [ current_reasoner_module/1,      % -ReasonerModule
      current_reasoner_manager/1,     % -ReasonerManager
      set_current_reasoner_module/1,  % +ReasonerModule
      reasoner_define_relation/2,     % +Relation, +Arity
      reasoner_setting/2,             % +Name, ?Value
      reasoner_setting/4,             % +Name, +Type, +Default, +Comment
      reasoner_set_setting/3,         % +ResonerModule, +Name, +Value
      reasoner_rdf_init/1,            % +ResonerModule
      reasoner_call/2
    ]).

/** <module> Query evaluation via a blackboard.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
        [ rdf_current_predicate/1,
          rdf_global_term/2,
          rdf/4 ]).
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

%% current_reasoner_manager(?ReasonerManager) is semidet.
%
current_reasoner_manager(ReasonerManager) :-
    nb_current(reasoner_manager, ReasonerManager),
    !.
current_reasoner_manager(0).

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

%% reasoner_define_relation(+Name, +Arity) is nondet.
%
% Define a relation in the current reasoner module.
%
reasoner_define_relation(Name, Arity) :-
    nonvar(Name), nonvar(Arity), !,
    current_reasoner_module(Reasoner),
    current_reasoner_manager(ReasonerManager),
    reasoner_define_relation_cpp(ReasonerManager, Reasoner, Name, Arity).

%% reasoner_call(+Goal, +QueryContext) is nondet.
%
% option(instantiations(List)): List is a list of predicate instantations
%
reasoner_call(Goal, QueryContext) :-
    % note: PrologReasoner does not support scoped queries
    current_reasoner_module(Reasoner),
    % record instantiated predicates into a list if requested
    (   option(predicates(Instantiations), QueryContext)
    ->  expand_instantiations(Goal, Expanded0, Instantiations)
    ;   Expanded0 = Goal
    ),
    % RDF predicates may appear as `P(S,O)` in queries, rewrite
    % such expressions to `triple(S,P,O)`.
    expand_rdf_predicates(Expanded0, Expanded),
    Reasoner:call(Expanded).

%%
expand_rdf_predicates(Goal, triple(S, P, O)) :-
    compound(Goal),
    Goal =.. [P, S, O],
    atom(P),
    atom_concat('http', _, P), !.
expand_rdf_predicates(Goal, Goal) :-
    is_list(Goal), !.
expand_rdf_predicates(Goal, Expanded) :-
    compound(Goal), !,
    Goal =.. [Functor | Args],
    expand_rdf_predicates1(Args, ExpandedArgs),
    Expanded =.. [Functor | ExpandedArgs].
expand_rdf_predicates(Goal, Goal).

%%
expand_rdf_predicates1([],[]).
expand_rdf_predicates1([X|Xs],[Y|Ys]) :-
    expand_rdf_predicates(X,Y),
    expand_rdf_predicates1(Xs,Ys).

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
    % NOTE: findall-family of predicates cannot change the list of instantiations in their goal argument.
    %   in fact, each predicate would need to appear in the pattern of the findall such that it is
    %   recorded. Hence, findall-family of predicates need in addition a rewriting of the pattern
    %   argument, adding the list of instantiations. but the meaning would not be clear. and also
    %   uses of the list would need to be rewritten. seems difficult.
    %   hence, currently instantations made during the evaluation of findall predicates
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
