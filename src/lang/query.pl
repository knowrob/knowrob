:- module(lang_query,
    [ ask(t),        % +Statement
      ask(t,t),      % +Statement, +Scope
      tell(t),       % +Statement
      tell(t,t),     % +Statement, +Scope
      universal_scope/1,
      subscope_of/2,
      scope_satisfies/2,
      scope_merge/3,
      scope_intersect/3,
      op(900, xfx, (?>)),
      op(900, xfx, (+>)),
      op(900, xfx, (?+>))
    ]).
/** <module> The querying language of KnowRob.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('utility/modules'),
    [ strip_module/2
    ]).

:- use_module('./scopes/temporal.pl',
    [ time_subscope_of/2,
      time_scope_intersect/3,
      time_scope_merge/3,
      time_scope_satisfies/2
    ]).

:- dynamic ask/2.
:- dynamic tell/2.

%% ask(+Statement) is nondet.
%
% Ask the knowledge base if some statement is true.
%
ask(Statement) :-
  ask(Statement,_{}->_).

%%
%
%
ask(Statements,QScope->FScope) :-
  is_list(Statements),!,
  ask_all_(Statements,QScope,_->FScope).

%%
ask_all_([],_QS,FS->FS) :- !.
ask_all_([X|Xs],QS->QSn,FS->FSn) :-
  ask(X,QS->FS0),
  scope_intersect(FS,FS0,FS1),
  ask_all_(Xs,QS,FS1->FSn).

%% tell(+Statement) is det.
%
% Tell the knowledge base that some statement is true.
% This will cause the statement to be added to the 
% tripledb used by the knowledge base.
%
tell(Statement) :-
  tell(Statement,_{}).

%%
%
%
tell(Statements,QScope) :-
  is_list(Statements),!,
  forall(
    member(Statement,Statements),
    tell(Statement,QScope)
  ).

		 /*******************************
		 *	   SCOPE	*
		 *******************************/

%%
%
%
universal_scope(Scope) :-
  % TODO: call scope modules to build universal scope
  Scope=_{ time: _{ since: double(0),
                    until: double('Infinity') }}.

%%
%
% True if Sup contains all facts that are contained
% in Sub.
%
subscope_of(Sub,Sup) :-
  %ground([Sub,Sup]),
  % TODO: handle other scopes
  get_dict(time,Sub,V0),
  get_dict(time,Sub,V1),
  time_subscope_of(V0,V1).

%%
%
%
scope_satisfies(Scope0,Scope1) :-
  %ground([Scope0,Scope1]),
  forall(
    ( get_dict(K,Scope0,V0) ),
    ( get_dict(K,Scope1,V1),
      scope_satisfies1(K,V0,V1)
    )).

scope_satisfies1(_K,V,V)     :- !.
scope_satisfies1(time,V0,V1) :- time_scope_satisfies(V0,V1).
scope_satisfies1(Key,_) :-
  % all cases need to be handled above
  throw(lang_error(unknown_scope(Key))).

%%
%
%
scope_merge(Scope0,Scope1,Merged) :-
  %ground([Scope0,Scope1]),
  findall(K-V, (
    get_dict(K,Scope0,V0),
    get_dict(K,Scope1,V1),
    scope_merge1(K,V0,V1,V)
  ), Pairs),
  dict_pairs(Merged,_,Pairs).

scope_merge1(_K,V,V,V)     :- !.
scope_merge1(time,V0,V1,V) :- time_scope_merge(V0,V1,V).
scope_merge1(Key,_,_,_) :-
  % all cases need to be handled above
  throw(lang_error(unknown_scope(Key))).

%%
%
%
scope_intersect(Scope0,Scope1,Intersection) :-
  %ground([Scope0,Scope1]),
  findall(K-V, (
    get_dict(K,Scope0,V0),
    get_dict(K,Scope1,V1),
    scope_intersect1(K,V0,V1,V)
  ),Pairs),
  dict_pairs(Intersection,_,Pairs).

scope_intersect1(_K,V,V,V)     :- !.
scope_intersect1(time,V0,V1,V) :- time_scope_intersect(V0,V1,V).
scope_intersect1(Key,_) :-
  % all cases need to be handled above
  throw(lang_error(unknown_scope(Key))).

%%
scope_update(Original,Inserted,Updated) :-
  findall(K-V, (
    ( get_dict(K,Inserted,V) ) ;
    ( get_dict(K,Original,V),
      \+ get_dict(K,Inserted,_)
    )
  ), Pairs),
  dict_pairs(Updated,_,Pairs).

%%
scope_remove(Original,Key,Value,Updated) :-
  get_dict(Key,Original,Value),!,
  findall(K-V, (
    get_dict(K,Original,V),
    K \= Key
  ), Pairs),
  dict_pairs(Updated,_,Pairs).
scope_remove(Scope0,_,_,Scope0).

		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%%
% Term expansion for *ask* rules using the (?>) operator.
% The head is rewritten as ask head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((?>(Head,Goal)),
        [ (:-(lang_query:ask(Term,QScope->FScope),GoalExpanded)),
          (:-(Head,ask(Term)))
        ]) :-
  strip_module(Head,Term),
  expand_ask_term_(QScope->_,_->FScope,
                   Goal,GoalExpanded).

%%
expand_ask_term_(QS0->QSn, FS0->FSn,
        (,(First0,Rest0)),
        (,(First1,Rest1))) :-
  expand_ask_term_(QS0->QS1,FS0->FS1,First0,First1),!,
  expand_ask_term_(QS1->QSn,FS1->FSn,Rest0,Rest1).
% call unscoped goal
expand_ask_term_(QS->QS, FS->FS, { Goal }, Goal ).
% get current scope
expand_ask_term_(QS->QS, FS->FS, `query-scope`(X), (X=QS)).
expand_ask_term_(QS->QS, FS->FS, `fact-scope`(X),  (X=FS)).
% interact with the query scope
expand_ask_term_(QS->Qx, FS->FS, `scope`(K,X),
    ( scope_update(QS,_{K:X},Qx) )).
expand_ask_term_(QS->Qx, FS->FS, `unscope`(K,X),
    ( scope_update(QS,K,X,Qx) )).
% call scoped predicate
expand_ask_term_(QS->QS, FS->Fx, `call`(X,Q0),
    ( scope_update(QS,Q0,Q1),
      lang_query:ask(X,Q1->F1),
      scope_intersect(FS,F1,Fx) )).
expand_ask_term_(QS->QS, FS->Fx, `call`(X),
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
  strip_module(Head,Term),
  expand_tell_term_(Scope->_,Goal,GoalExpanded).

%%
%
expand_tell_term_(QS0->QSn,
        (,(First0,Rest0)),
        (,(First1,Rest1))) :-
  expand_tell_term_(QS0->QS1,First0,First1),!,
  expand_tell_term_(QS1->QSn,Rest0,Rest1).
% call unscoped goal
expand_tell_term_(QS->QS, { Goal }, Goal ).
% get the scope
expand_tell_term_(QS->QS, `fact-scope`(X), (X=QS)).
% interact with the query scope
expand_tell_term_(QS->Qx, `scope`(K,X),
    ( scope_update(QS,_{K:X},Qx) )).
expand_tell_term_(QS->Qx, `unscope`(K,X),
    ( scope_remove(QS,K,X,Qx) )).
% notifications
% TODO: would be nice to notify at the end
%           of a query, not instantly
expand_tell_term_(QS->QS, notify(X), notify(X)).
% call scoped predicate
expand_tell_term_(QS->QS, `call`(Statement,Q0),
    ( scope_update(QS,Q0,Q1),
      lang_query:tell(Statement,Q1) )).
expand_tell_term_(QS->QS, `call`(Statement),
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
  user:term_expansion((?>(Head,Goal),X1)),
  user:term_expansion((+>(Head,Goal),X2)).
