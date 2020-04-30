:- module(spatial_distance,
    [ object_distance(r,r,?)
    ]).

% TODO: consider qualitative distance: close-to, far-away-from, ...

%% object_distance(+A:iri, +B:iri, ?Distance:float) is semidet
% 
% Computes euclidean distance between A and B.
%
% @param A         Instance of SpatialThing
% @param B         Instance of SpatialThing
% @param Distance  The current distance between A and B
%
object_distance(A,B,Distance) ?>
  % FIXME: hardcoded map
  is_at(A, [map,[AX,AY,AZ],_]),
  is_at(B, [map,[BX,BY,BZ],_]),
  { DX is AX - BX,
    DY is AY - BY,
    DZ is AZ - BZ,
    Distance is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ))
  }.
