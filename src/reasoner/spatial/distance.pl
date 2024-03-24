:- module(spatial_distance,
    [ object_distance(r,r,?)
    ]).

%% object_distance(+A:iri, +B:iri, ?Distance:float) is semidet
% 
% Computes euclidean distance between A and B.
%
% @param A         Instance of SpatialThing
% @param B         Instance of SpatialThing
% @param Distance  The current distance between A and B
%
object_distance(A,B,Distance) :-
	ground(A),
	ground(B),
	map_origin_frame(MapFrame),
	is_at(A, [MapFrame,[AX,AY,AZ],_]),
	is_at(B, [MapFrame,[BX,BY,BZ],_]),
	DX is AX - BX,
	DY is AY - BY,
	DZ is AZ - BZ,
	Distance is sqrt(((DX*DX) + (DY*DY)) + (DZ*DZ)).
