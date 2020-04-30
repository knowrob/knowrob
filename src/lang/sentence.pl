:- module(lang_sentence,
    [ ask(t),        % +Statement
      ask(t,t),      % +Statement, +Scope
      tell(t),       % +Statement
      tell(t,t),     % +Statement, +Scope
      op(900, xfx, (?>)),
      op(900, xfx, (+>)),
      op(900, xfx, (?+>))
    ]).
/** <module> The querying language of KnowRob.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('utility/modules'),
        [ strip_module/2 ]).
:- use_module('scopes/propagation',
        [ scope_propagate/2 ]).

:- dynamic ask/2.
:- dynamic tell/2.

%%
% Term expansion for *ask* rules using the (?>) operator.
% The head is rewritten as ask head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((?>(Head,Goal)),
        [ (:-(HeadExpanded,GoalExpanded)),
          (:-(Head,ask(Term)))
        ]) :-
  strip_module(Head,Term),
  HeadExpanded=( lang_api:ask(Term,Scope) ),
  expand_scoped_term_(Scope,lang_api:ask,Goal,GoalExpanded).

%%
% Term expansion for *tell* rules using the (+>) operator.
% The head is rewritten as tell/2 head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((+>(Head,Goal)),
                    (:-(HeadExpanded,GoalExpanded))) :-
  %%
  strip_module(Head,Term),
  HeadExpanded=( lang_api:tell(Term,Scope) ),
  expand_scoped_term_(Scope,lang_api:tell,Goal,GoalExpanded).

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

%% ask(+Statement) is nondet.
%
% Ask the knowledge base if some statement is true.
%
ask(Statement) :-
  ask(Statement,_{}).

%% tell(+Statement) is det.
%
% Tell the knowledge base that some statement is true.
% This will cause the statement to be added to the 
% tripledb used by the knowledge base.
%
% TODO: auto-stop fact if property is functional.
%
tell(Statement) :-
  tell(Statement,_{}).

		 /*******************************
		 *	   SCOPE	*
		 *******************************/

%%
expand_scoped_term_(S,P,
        (,(First0,Rest0)),
        (,(First1,Rest1))) :-
  expand_scoped_term_(S,P,First0,First1),!,
  expand_scoped_term_(S,P,Rest0,Rest1).

% call regular predicates
expand_scoped_term_(_,  _, { X }, X ).

% get the scope
expand_scoped_term_(S,  _, `scope`(X), (X=S)).

% add a scope
expand_scoped_term_(S,  _, `scope`(K,X), scope_propagate(S,_{K:X})).

% remove a scope
expand_scoped_term_(S,  _, `unscope`(K,X), scope_remove(S,K,X)).

% call scoped predicate with additional scope
expand_scoped_term_(S0, P, `call`(X,S1), Term) :-
  GoalTerm=..[P,X,S1],
  Term=( scope_propagate(S1,S0), GoalTerm ).

% call scoped predicate without additional scope
expand_scoped_term_(S,  P, `call`(X), Term) :- Term=..[P,X,S].

% call scoped predicate without additional scope
expand_scoped_term_(S, P, X, Term) :- Term=..[P,X,S].

%%
scope_remove(QScope->_,X,Y) :- !,
  scope_remove(QScope,X,Y).

scope_remove(Scope,Key,RemovedValue) :-
  ( get_dict(Key,Scope,RemovedValue) ->
    b_set_dict(Key,Scope,_);
    RemovedValue=_
  ).
