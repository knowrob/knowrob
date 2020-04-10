
:- module(model_OWL,
    [
      is_class(r),
      is_individual(r),
      is_object_property(r),
      is_data_property(r)
    ]).
:- rdf_module.
/** <module> TODO ...

@author Daniel Beßler
*/

%% is_class(+Entity) is semidet.
%
% True iff Entity is a class IRI.
%
% @param Entity An entity IRI.
%
is_class(Entity) :-
  ask( Entity rdf:type owl:'Class' ),!.

%% is_individual(+Entity) is semidet.
%
% True iff Entity is an individual IRI.
%
% @param Entity An entity IRI.
%
is_individual(Entity) :-
  ask( Entity rdf:type owl:'NamedIndividual' ),!.

%% is_object_property(+Entity) is semidet.
%
% True iff Entity is an object property IRI.
%
% @param Entity An entity IRI.
%
is_object_property(Entity) :-
  ask( Entity rdf:type owl:'ObjectProperty' ),!.

%% is_data_property(+Entity) is semidet.
%
% True iff Entity is an datatype property IRI.
%
% @param Entity An entity IRI.
%
is_data_property(Entity) :-
  ask( Entity rdf:type owl:'DatatypeProperty' ),!.
