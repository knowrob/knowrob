
:- module('knowrob/triples/triple_store',
    [
      vkb_has_type/3,
      vkb_has_triple/4
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/triples/computable'), [rdfs_compute/4]).

:- rdf_meta vkb_has_triple(r,r,t,-),
            vkb_has_type(r,r,-).

%%
:- multifile vkb_has_triple/4,
             vkb_has_type/3.

%% vkb_has_triple(?S,?P,?O,+DBArgs) is nondet.
%
% This is a multifile predicate that can be extended
% in KnowRob packages to include additional knowledge sources.
%
vkb_has_triple(S,P,O,DBArgs) :-
  triple_db_has_property(S,P,O,DBArgs).

vkb_has_triple(S,P,O,Args) :-
  rdfs_compute(S,P,O,Args).

%% vkb_has_type(?S,?Type,+DBArgs) is nondet.
%
% This is a multifile predicate that can be extended
% in KnowRob packages to include additional knowledge sources.
%
vkb_has_type(S,Type,DBArgs) :-
  triple_db_has_type(S,Type,DBArgs).
  
vkb_has_type(S,Type,DBArgs) :-
  rdf_equal(rdf:type,Property),
  ( nonvar(Type) ->
    rdfs_subclass_of(SubType, Type) ;
    SubType = Type ),
  vkb_has_triple(S,Property,SubType,DBArgs).

%%
triple_db_has_property(S,P,O,_DBArgs) :-
  (  atom(O), rdf_has(P, rdf:type, owl:'DatatypeProperty') )
  -> rdf_has(S,P,literal(type(_,O)))
  ;  rdf_has(S,P,O).

%%
triple_db_has_type(Resource,Type,_DBArgs) :-
  rdfs_individual_of(Resource,Type).
