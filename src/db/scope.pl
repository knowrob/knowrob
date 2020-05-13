:- module(scope,
    [ universal_scope/1,
      wildcard_scope/1,
      subscope_of/2,
      scope_satisfies/2,
      scope_merge/3,
      scope_intersect/3,
      scope_update/3,
      scope_remove/4,
      scope_query_overlaps/2
    ]).
/** <module> TODO

@author Daniel Beßler
@license BSD
*/

:- multifile universal_scope/2,
             subscope_of/3,
             scope_merge/4,
             scope_intersect/4,
             scope_satisfies/3,
             scope_query_overlaps/3.

%%
% The scope of facts that are universally true.
%
universal_scope(Scope) :-
  current_predicate(universal_scope_/1),
  universal_scope_(Scope),!.

universal_scope(Scope) :-
  findall(K-V,
    universal_scope(K,V),
    Pairs),
  dict_pairs(Scope,_,Pairs),
  assertz(universal_scope_(Scope)).

%%
% TODO: should we always specify full scope instead for seacrh indices to work?
%         - at least for tell it is slower to use search indices on the scope :/
%
wildcard_scope(_{}).

%%
%
% True if Sup contains all facts that are contained
% in Sub.
%
subscope_of(Sub,Sup) :-
  %ground([Scope0,Scope1]),
  forall(
    ( get_dict(K,Sub,V0) ),
    ( get_dict(K,Sup,V1),
      subscope_of1(K,V0,V1)
    )).

subscope_of1(_K,V,V)  :- !.
subscope_of1(K,V0,V1) :- subscope_of(K,V0,V1), !.
subscope_of1(K,_,_)   :- throw(lang_error(unknown_scope(K))).

%%
%
%
scope_satisfies(_,_{}) :- !.
scope_satisfies(Scope0,Scope1) :-
  %ground([Scope0,Scope1]),
  forall(
    % TODO: what about K missing in Scope1? I think any
    %       scope satisfies then!
    ( get_dict(K,Scope0,V0) ),
    ( get_dict(K,Scope1,V1),
      scope_satisfies1(K,V0,V1)
    )).

scope_satisfies1(_K,V,V)  :- !.
scope_satisfies1(K,V0,V1) :- scope_satisfies(K,V0,V1), !.
scope_satisfies1(K,_,_)   :- throw(lang_error(unknown_scope(K))).

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

scope_merge1(_K,V,V,V)  :- !.
scope_merge1(K,V0,V1,V) :- scope_merge(K,V0,V1,V), !.
scope_merge1(K,_,_,_)   :- throw(lang_error(unknown_scope(K))).

%%
%
%
scope_intersect(_{},Scope,Scope) :- !.
scope_intersect(Scope,_{},Scope) :- !.
scope_intersect(Scope,Scope,Scope) :- !.
scope_intersect(Scope0,Scope1,Intersection) :-
  %ground([Scope0,Scope1]),
  findall(K-V, (
    get_dict(K,Scope0,V0),
    get_dict(K,Scope1,V1),
    scope_intersect1(K,V0,V1,V)
  ),Pairs),
  dict_pairs(Intersection,_,Pairs).

scope_intersect1(_K,V,V,V)  :- !.
scope_intersect1(K,V0,V1,V) :- scope_intersect(K,V0,V1,V), !.
scope_intersect1(K,_,_,_)   :- throw(lang_error(unknown_scope(K))).

%%
%
%
scope_query_overlaps(FScope,QScope) :-
  findall(K-V, (
    get_dict(K,FScope,V0),
    scope_query_overlaps1(K,V0,V)
  ),Pairs),
  dict_pairs(QScope,_,Pairs).

scope_query_overlaps1(K,V0,V) :- scope_query_overlaps(K,V0,V), !.
scope_query_overlaps1(K,_,_)  :- throw(lang_error(unknown_scope(K))).

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
