:- module(blackboard,
    [ current_reasoner_module/1,      % -ReasonerModule
      set_current_reasoner_module/1,  % +ReasonerModule
      reasoner_setting/2,             % +Name, ?Value
      reasoner_setting/4,             % +Name, +Type, +Default, +Comment
      reasoner_set_setting/3,         % +ResonerModule, +Name, +Value
      reasoner_rdf_init/1,            % +ResonerModule
      prolog_call/2
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

%%
%
prolog_call(Goal, _QueryContext) :-
    % note: PrologReasoner does not support scoped queries
    call(Goal).

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
