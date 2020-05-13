:- module(lang_query,
    [ ask(t),     % +Statement
      ask(t,t),   % +Statement, +Scope
      tell(t),    % +Statement
      tell(t,t)   % +Statement, +Scope
    ]).
/** <module> Main interface predicates for querying the knowledge base.

@author Daniel Beßler
@license BSD
*/

:- op(1100, xfx, user:(?>)).
:- op(1100, xfx, user:(+>)).
:- op(1100, xfx, user:(?+>)).

:- use_module(library('db/scope'),
    [ scope_intersect/3,
      scope_update/3,
      scope_remove/4
    ]).

:- multifile ask/2, tell/2.

%% ask(+Statement) is nondet.
%
% Same as ask/2 with wildcard scope.
%
% @param Statement a statement term.
%
ask(Statement) :-
  wildcard_scope(QScope),
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
ask(Statements,QScope->FScope) :-
  is_list(Statements),!,
  ask_all_(Statements,QScope,_->FScope).

ask(triple(S,P,O),[Options,QScope]->FScope) :-
  tripledb_ask(S,P,O,QScope,FScope,Options).

ask_all_([],_QS,FS->FS) :- !.
ask_all_([X|Xs],QS,FS->FSn) :-
  ask(X,QS->FS0),
  % TODO: fact scope should restrict query scope for next query!
  scope_intersect(FS,FS0,FS1),
  ask_all_(Xs,QS,FS1->FSn).

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
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
tell(Statements,Scope) :-
  is_list(Statements),!,
  tell_all(Statements,Scope).

tell(triple(S,P,O),QScope) :-
  is_dict(QScope),!,
  tell(triple(S,P,O),[[],QScope]).

tell(triple(S,P,O),[Options,QScope]) :-
  tripledb_tell(S,P,O,QScope,Options).

tell_all([],_) :- !.
tell_all([X|Xs],QScope) :-
  tell(X,QScope),
  tell_all(Xs,QScope).

		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%%
% Term expansion for *ask* rules using the (?>) operator.
% The head is rewritten as ask head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((?>(Head,Goal)),Expansions) :-
  strip_module_(Head,Module,Term),
  once((ground(Module);prolog_load_context(module,Module))),
  %%
  findall(Expansion, (
    expand_predicate_(Module,Term,Expansion);
    expand_ask_query_(Term,Goal,Expansion)
  ), Expansions).

%%
% Expand `f(..) ?> ...` to `f(..) :- lang_query:ask(f(..))`
%
expand_predicate_(Module,Head,(:-(HeadExpanded,lang_query:ask(Term)))) :-
  functor(Head,Functor,Arity),
  Indicator=(:(Module,(/(Functor,Arity)))),
  \+ current_predicate(Indicator),
  length(Args,Arity),
  Term=..[Functor|Args],
  HeadExpanded=(:(Module,Term)).

%%
expand_ask_query_(Head,Goal,(:-(HeadExpanded,GoalExpanded))) :-
  expand_ask_term_(QScope->_,_->FScope,
                   Goal,GoalExpanded),
  HeadExpanded = lang_query:ask(Head,QScope->FScope),
  !.

%%
expand_ask_term_(QS0->QSn, FS0->FSn,
        (','(First0,Rest0)),
        (','(First1,Rest1))) :-
  !,
  expand_ask_term_(QS0->QS1,FS0->FS1,First0,First1),
  expand_ask_term_(QS1->QSn,FS1->FSn,Rest0,Rest1).
% call unscoped goal
expand_ask_term_(QS->QS, FS->FS, { Goal }, Goal ).
% options
expand_ask_term_(QS->QS, FS->FS, options(X), (QS=[X,_])).
expand_ask_term_([Opt0,QS]->[Opt1,QS], FS->FS, option(X),
    ( merge_options([X],Opt0,Opt1)) ).
% scope
expand_ask_term_(QS->QS, FS->FS, query_scope(X), (QS=[_,X])).
expand_ask_term_(QS->QS, FS->FS, fact_scope(X),  (FS=X)).
expand_ask_term_([Cfg,QS]->[Cfg,Qx], FS->FS, scope(K,X),
    ( scope_update(QS,K,X,Qx) )).
expand_ask_term_([Cfg,QS]->[Cfg,Qx], FS->FS, unscope(K,X),
    ( scope_update(QS,K,X,Qx) )).
% call scoped predicate
expand_ask_term_(QS->QS, FS->Fx, call(X,Q0),
    ( lang_query:context_update_(QS,Q0,Q1),
      lang_query:ask(X,Q1->F1),
      scope_intersect(FS,F1,Fx) )).
expand_ask_term_(QS->QS, FS->Fx, call(X),
    ( lang_query:ask(X,QS->F1),
      scope_intersect(FS,F1,Fx) )).
expand_ask_term_(QS->QS, FS->Fx, Goal,
    ( lang_query:ask(Goal,QS->F1),
      scope_intersect(FS,F1,Fx) )).

%%
% Term expansion for *tell* rules using the (+>) operator.
% The head is rewritten as tell/2 head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((+>(Head,Goal)),
                    (:-(lang_query:tell(Term,Scope),GoalExpanded))) :-
  %%
  strip_module_(Head,_,Term),
  expand_tell_term_(Scope->_,Goal,GoalExpanded).

%%
%
expand_tell_term_(QS0->QSn,
        (','(First0,Rest0)),
        (','(First1,Rest1))) :-
  !,
  expand_tell_term_(QS0->QS1,First0,First1),
  expand_tell_term_(QS1->QSn,Rest0,Rest1).
% call unscoped goal
expand_tell_term_(QS->QS, { Goal }, Goal ).
% options
expand_tell_term_(QS->QS, options(X), (QS=[X,_])).
expand_tell_term_([Opt0,QS]->[Opt1,QS], option(X),
    ( merge_options([X],Opt0,Opt1)) ).
% scope
expand_tell_term_(QS->QS, fact_scope(X), (QS=[_,X])).
expand_tell_term_([Opt,QS]->[Opt,Qx], scope(K,X),
    ( scope_update(QS,K,X,Qx) )).
expand_tell_term_([Opt,QS]->[Opt,Qx], unscope(K,X),
    ( scope_remove(QS,K,X,Qx) )).
% notifications
% TODO: would be nice to notify at the end
%           of a query, not instantly
expand_tell_term_(QS->QS, notify(X), notify(X)).
% call scoped predicate
expand_tell_term_(QS->QS, call(Statement,Q0),
    ( lang_query:context_update_(QS,Q0,Q1),
      lang_query:tell(Statement,Q1) )).
expand_tell_term_(QS->QS, call(Statement),
    ( lang_query:tell(Statement,QS) )).
expand_tell_term_(QS->QS, Statement,
    ( lang_query:tell(Statement,QS) )).

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

%%
% NOTE: SWI strip_module acts strange
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).

%%
context_update_([Options0,Scope0],New,[Options1,Scope1]) :-
  ( option(scope(NewScope),New) ->
    scope_update(Scope0,NewScope,Scope1) ;
    Scope1=Scope0
  ),
  ( option(options(NewOptions),New) ->
    merge_options(NewOptions,Options0,Options1) ;
    Options1=Options0
  ).
