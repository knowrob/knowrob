
:- module(model_DUL_Parameter,
    [
      is_parameter(r),   % ?Parameter
      has_parameter(r,r) % ?Parameter, ?Object
    ]).
:- rdf_module.
/** <module> DUL notion of Parameter.

In DUL, Parameter is defined as:
  "A Concept that classifies a Region; the difference between a Region and a Parameter is that regions represent sets of observable values, e.g. the height  of a given building, while parameters represent constraints or selections on observable values, e.g. 'VeryHigh'. Therefore, parameters can also be used to constrain regions, e.g. VeryHigh on a subset of values of the Region Height applied to buildings, or to add an external selection criterion, such as measurement units, to regions, e.g. Meter on a subset of values from the Region Length applied to the Region Length applied to roads."

@author Daniel Beßler
@license BSD
*/

%% is_parameter(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Parameter'.
%
% @param Entity An entity IRI.
%
is_parameter(Entity) :-
  ask( Entity rdf:type dul:'Parameter' ).

%%
%
has_parameter(Entity,Parameter) :-
  ask( Parameter dul:classifies Entity ).
