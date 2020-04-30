:- module(scope_propagation,
    [ scope_propagate(t,t)
    ]).
/** <module> ....

@author Daniel Beßler
@license BSD
*/

%% scope_propagate(+Scope0,+Scope1) is det.
%
%
scope_propagate(QScope->_,X) :- !,
  scope_propagate(QScope,X).

scope_propagate(_,_{}) :- !.

scope_propagate(ScopeToModify,ScopeToAdd) :-
  forall(
    get_dict(K,ScopeToAdd,V),
    scope_propagate1(ScopeToModify,K,V)
  ).

scope_propagate1(Scope,Key,New) :-
  get_dict(Key,ScopeToModify,Old),!,
  scope_propagate2(Key,Old,New).

scope_propagate1(Scope,Key,New) :-
  dict_copy_(New,X),
  b_set_dict(Key,Scope,X).

% values are the same already
scope_propagate2(_,V,V) :- !.

% ignore unit scope
scope_propagate2(unit,_,_) :- !.

% handle time scope
scope_propagate2(time,Old,New) :- !,
  %%
  ( get_dict(since,New,S1) ->
    ( get_dict(since,Old,S0) ->
      ( S is max(Since0,S1), b_set_dict(since,Old,S) );
      ( b_set_dict(since,Old,S1) ));
    ( true )
  ),
  %%
  ( get_dict(until,New,U1) ->
    ( get_dict(until,Old,U0) ->
      ( U is min(U0,U1),  b_set_dict(until,Old,U) );
      ( b_set_dict(until,Old,U1) ));
    ( true )
  ).

% ....
scope_propagate2(Key,_,_) :-
  % all cases need to be handled above
  throw(lang_error(unknown_scope(Key))).

%%
dict_copy_(IN,IN) :-
  \+ is_dict(IN), !.

dict_copy_(IN,OUT) :-
  forall( get_dict(K,IN,V), (
    dict_copy_(V,X),
    b_set_dict(K,OUT,X)
  )).
