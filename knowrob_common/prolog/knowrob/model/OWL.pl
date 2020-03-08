
:- module('knowrob/model/OWL',
    [
      kb_is_class/1,
      kb_is_individual/1,
      kb_is_object_property/1,
      kb_is_data_property/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_meta kb_is_class(r),
            kb_is_individual(r),
            kb_is_object_property(r),
            kb_is_data_property(r).

%% kb_is_class(+Entity) is semidet.
%
% True iff Entity is a class IRI.
%
% @param Entity An entity IRI.
%
kb_is_class(Entity) :-
  atom(Entity),
  rdfs_individual_of(Entity, owl:'Class'),!.

%% kb_is_individual(+Entity) is semidet.
%
% True iff Entity is an individual IRI.
%
% @param Entity An entity IRI.
%
kb_is_individual(Entity) :-
  atom(Entity),
  rdfs_individual_of(Entity, owl:'NamedIndividual'),!.

%% kb_is_object_property(+Entity) is semidet.
%
% True iff Entity is an object property IRI.
%
% @param Entity An entity IRI.
%
kb_is_object_property(Entity) :-
  atom(Entity),
  rdfs_individual_of(Entity, owl:'ObjectProperty'),!.

%% kb_is_data_property(+Entity) is semidet.
%
% True iff Entity is an datatype property IRI.
%
% @param Entity An entity IRI.
%
kb_is_data_property(Entity) :-
  atom(Entity),
  rdfs_individual_of(Entity, owl:'DatatypeProperty'),!.
