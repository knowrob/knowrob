:- module(spatial_topogical,
    [ shape_contains(r,r) %-> knowrob:isInsideOf
    ]).
/** <module> Inferring relations of the Dimensionally Extended nine-Intersection Model (DE-9IM).

@author Daniel Beßler
@license BSD
*/

%% shape_contains(?InnerObj, ?OuterObj) is nondet.
%
% Check if InnerObj is contained by OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
%
shape_contains(InnerObj, OuterObj) :-
  ground(InnerObj),
  ground(OuterObj),
  % FIXME: hardcoded map
  is_at(InnerObj, [map, [IX,IY,IZ], _]),
  is_at(OuterObj, [map, [OX,OY,OZ], _]),
  InnerObj \== OuterObj,
  %
  object_dimensions(InnerObj, ID, IW, IH),
  object_dimensions(OuterObj, OD, OW, OH),
  % InnerObj is contained by OuterObj if (center_i+0.5*dim_i)<=(center_o+0.5*dim_o)
  % for all dimensions (x, y, z)
  >=( (IX - 0.5*ID), (OX - 0.5*OD)-0.05 ),
  =<( (IX + 0.5*ID), (OX + 0.5*OD)+0.05 ),
  >=( (IY - 0.5*IW), (OY - 0.5*OW)-0.05 ),
  =<( (IY + 0.5*IW), (OY + 0.5*OW)+0.05 ),
  >=( (IZ - 0.5*IH), (OZ - 0.5*OH)-0.05 ),
  =<( (IZ + 0.5*IH), (OZ + 0.5*OH)+0.05 ).

%% shape_within(+A,?B) is nondet
%shape_within(A,B) :-
  %shape_contains(B,A).

%% shape_intersects(+A,?B) is nondet
%shape_intersects(A,B) :-
  %fail.

%% shape_touches(+A,?B) is nondet
%shape_touches(A,B) :-
  %fail.

%% shape_covers(+A,?B) is nondet
%shape_covers(A,B) :-
  %fail.

%% shape_covered_by(+A,?B) is nondet
%shape_covered_by(A,B) :-
  %shape_covers(B,A).

%% shape_equal(+A,?B) is nondet
%shape_equal(A,B) :-
  %fail.

%% shape_disjoint(+A,?B) is nondet
%shape_disjoint(A,B) :-
  %fail.
