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
  map_origin_frame(MapFrame),
  is_at(InnerObj, [MapFrame, [IX,IY,IZ], _]),
  is_at(OuterObj, [MapFrame, [OX,OY,OZ], _]),
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
