
:- module(model_DUL_Quality,
    [
      is_quality(r)  % ?Quality
    ]).
:- rdf_module.
/** <module> DUL notion of Quality.

In DUL, Quality is defined as:
  "Any aspect of an Entity (but not a part of it), which cannot exist without that Entity."

@author Daniel Beßler
@license BSD
*/

%% is_quality(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Quality'.
%
% @param Entity An entity IRI.
%
is_quality(Entity) :-
  ask( Entity rdf:type dul:'Quality' ).
