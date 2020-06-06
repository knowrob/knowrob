:- module(scope,
    [ universal_scope/1,
      wildcard_scope/1,
      current_scope/1,
      subscope_of/2,
      scope_merge/3,
      scope_intersect/3,
      scope_overlaps_query/2,
      scope_update/3,
      scope_remove/4
    ]).
/** <module> The scope of statements being true.

@author Daniel Beßler
@license BSD
*/

:- multifile universal_scope/2,
             current_scope/2,
             subscope_of/3,
             scope_merge/4,
             scope_intersect/4,
             scope_overlaps_query/3.

%% universal_scope(-Scope) is det.
%
% The scope of facts that are universally true.
%
% @param Scope A scope dictionary.
%
universal_scope(Scope) :-
	current_predicate(universal_scope_/1),
	universal_scope_(Scope),!.

universal_scope(Scope) :-
	findall(K-V,
		universal_scope(K,V),
		Pairs
	),
	dict_pairs(Scope,_,Pairs),
	assertz(universal_scope_(Scope)).

%% current_scope(-Scope) is det.
%
% The scope of facts that are currently true.
%
% @param Scope A scope dictionary.
%
current_scope(Scope) :-
	findall(K-V,
		current_scope(K,V),
		Pairs
	),
	dict_pairs(Scope,_,Pairs).

%% wildcard_scope(-Scope) is det.
%
% A scope that matches any fact.
%
% @param Scope A scope dictionary.
%
% TODO: should we always specify full scope instead for seacrh indices to work?
%         - at least for tell it is slower to use search indices on the scope :/
wildcard_scope(_{}).

%% subscope_of(+Sub,+Sup) is det.
%
% True if scope Sup contains all facts that are contained
% in scope Sub.
%
% @param Sub a scope dict.
% @param Sup a scope dict.
%
subscope_of(Sub,Sup) :-
	%ground([Scope0,Scope1]),
	forall(
		( get_dict(K,Sub,V0) ),
		( get_dict(K,Sup,V1), subscope_of1(K,V0,V1) )
	).

subscope_of1(_K,V,V)  :- !.
subscope_of1(K,V0,V1) :- subscope_of(K,V0,V1), !.
subscope_of1(K,_,_)   :- throw(lang_error(unknown_scope(K))).

%% scope_merge(+A,+B,-Merged) is det.
%
% Merge two scopes. The merged scope contains all
% facts that are contained in scope A or scope B.
%
% @param A a scope dict.
% @param B a scope dict.
%
scope_merge(Scope0,Scope1,Merged) :-
	%ground([Scope0,Scope1]),
	findall(K-V,
		(	get_dict(K,Scope0,V0),
			get_dict(K,Scope1,V1),
			scope_merge1(K,V0,V1,V)
		),
		Pairs
	),
	dict_pairs(Merged,_,Pairs).

scope_merge1(_K,V,V,V)  :- !.
scope_merge1(K,V0,V1,V) :- scope_merge(K,V0,V1,V), !.
scope_merge1(K,_,_,_)   :- throw(lang_error(unknown_scope(K))).

%% scope_intersect(+A,+B,-Merged) is det.
%
% Intersect two scopes. The intersected scope contains all
% facts that are contained in scope A and also in scope B.
%
% @param A a scope dict.
% @param B a scope dict.
%
scope_intersect(_{},Scope,Scope) :- !.
scope_intersect(Scope,_{},Scope) :- !.
scope_intersect(Scope,Scope,Scope) :- !.
scope_intersect(Scope0,Scope1,Intersection) :-
	%ground([Scope0,Scope1]),
	findall(K-V,
		(	get_dict(K,Scope0,V0),
			get_dict(K,Scope1,V1),
			scope_intersect1(K,V0,V1,V)
		),
		Pairs
	),
	dict_pairs(Intersection,_,Pairs).

scope_intersect1(_K,V,V,V)  :- !.
scope_intersect1(K,V0,V1,V) :- scope_intersect(K,V0,V1,V), !.
scope_intersect1(K,_,_,_)   :- throw(lang_error(unknown_scope(K))).

%% scope_overlaps_query(+A,-B) is semidet.
%
% B is the scope of all facts whose scope overlaps with A.
%
scope_overlaps_query(FScope,QScope) :-
	findall(K-V,
		(	get_dict(K,FScope,V0),
			scope_overlaps_query1(K,V0,V)
		),
		Pairs
	),
	dict_pairs(QScope,_,Pairs).

scope_overlaps_query1(K,V0,V) :- scope_overlaps_query(K,V0,V), !.
scope_overlaps_query1(K,_,_)  :- throw(lang_error(unknown_scope(K))).

%% scope_update(+Original,+Inserted,-Updated) is det.
%
% Update a scope with new values taken from another scope
% dictionary. In case keys appear in both scopes, only
% the value from Inserted is used in Updated scope.
%
% @param Original a scope dictionary.
% @param Inserted a scope dictionary.
% @param Updated a scope dictionary.
%
scope_update(Original,Inserted,Updated) :-
	findall(K-V,
		(	( get_dict(K,Inserted,V) )
		;	( get_dict(K,Original,V), \+ get_dict(K,Inserted,_) )
		),
		Pairs
	),
	dict_pairs(Updated,_,Pairs).

%% scope_remove(+Original,+Key,-Value,-Updated) is det.
%
% Removes a key from a scope dictionary.
%
% @param Original a scope dictionary.
% @param Key a key in the scope dictionary.
% @param Value the value popped out of the dictionary.
% @param Updated the scope dictionary without Key.
%
scope_remove(Original,Key,Value,Updated) :-
	get_dict(Key,Original,Value),!,
	findall(K-V,
		(	get_dict(K,Original,V),
			K \= Key
		),
		Pairs
	),
	dict_pairs(Updated,_,Pairs).
scope_remove(Scope0,_,_,Scope0).
