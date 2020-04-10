
:- module(model_DUL_Role,
    [
      is_role(r),   % ?Role
      has_role(r,r) % ?Object, ?Role
    ]).
:- rdf_module.
/** <module> DUL notion of Role.

In DUL, Role is defined as:
  "A Concept that classifies an Object."

@author Daniel Beßler
@license BSD
*/

%% is_role(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Role'.
%
% @param Entity An entity IRI.
%
is_role(Entity) :-
  ask( Entity rdf:type dul:'Role' ).

%%
%
has_role(Entity,Role) :-
  ask( Role dul:classifies Entity ).
