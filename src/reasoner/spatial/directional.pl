:- module(spatial_directional,
    [ is_ontop_of(r,r)     %-> knowrob:isOntopOf
    , is_below_of(r,r)     %-> knowrob:isBelowOf
    , is_above_of(r,r)     %-> knowrob:isAboveOf
    , is_centered_at(r,r)  %-> knowrob:isInCenterOf
    %, is_infront_of(r,r)   -> knowrob:isInFrontOf
    %, is_right_of(r,r)     -> knowrob:isRightOf
    %, is_left_of(r,r)      -> knowrob:isLeftOf
    %, is_next_to(r,r)      -> knowrob:isNextTo
    ]).
/** <module> Predicates for spatial reasoning

@author Daniel Beßler
@license BSD
*/

%% is_ontop_of(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%
is_ontop_of(Top, Bottom) :-
	ground(Top),
	ground(Bottom),
	map_origin_frame(MapFrame),
	is_at(Top,    [MapFrame, [_,_,TZ], _]),
	is_at(Bottom, [MapFrame, [_,_,BZ], _]),
	Top \== Bottom,
	% the criterion is if the difference between them is less than epsilon=5cm
	Dist is TZ-BZ,
	Dist >= 0.0,
	Dist =< 0.05.

%% is_above_of(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%
is_above_of(Top, Bottom) :-
	ground(Top),
	ground(Bottom),
	map_origin_frame(MapFrame),
	is_at(Top,    [MapFrame, [_,_,TZ], _]),
	is_at(Bottom, [MapFrame, [_,_,BZ], _]),
	Top \== Bottom,
	<( BZ, TZ).

%% is_below_of(?Bottom, ?Top) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Bottom Identifier of the lower Object
% @param Top Identifier of the upper Object
%
is_below_of(Bottom, Top) :-
	is_above_of(Top, Bottom).

%% is_centered_at(?Inner, ?Outer) is nondet.
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
%
is_centered_at(Inner, Outer) :-
	ground(Inner),
	ground(Outer),
	map_origin_frame(MapFrame),
	is_at(Inner, [MapFrame, [IX,IY,IZ], _]),
	is_at(Outer, [MapFrame, [OX,OY,OZ], _]),
	Inner \== Outer,
	% less than 20cm x/y/z diff
	=<( abs( IX - OX), 0.20),
	=<( abs( IY - OY), 0.20),
	=<( abs( IZ - OZ), 0.20).

%% is_left_of(?Left, ?Right) is nondet.
%
% Check if Left is to the left of Right.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Left Identifier of the left Object
% @param Right Identifier of the right Object
%
%is_left_of(Left, Right, Scope) :-
    %physical_object(Left),
    %map_frame_name(MapFrame),
    %object_pose(Left, [MapFrame, _, [LX,LY,LZ], _], Time),
    %physical_object(Right),
    %Left \= Right,
    %object_pose(Right, [MapFrame, _, [RX,RY,RZ], _], Time),
    %=<( abs( LX - RX), 0.30),  % less than 30cm y diff
    %=<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
    %=<( abs( LZ - RZ), 0.30).  % less than 30cm height diff

%% is_right_of(?Right,?Left) is nondet.
%
% Check if Right is to the right of Left.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @see is_left_of
%
%is_right_of(Right, Left, Scope) :- is_left_of(Left, Right, Scope).

%% is_next_to(?A, ?B) is nondet.
%
% Check if A is either to the left or the rigth of B.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @see is_left_of
% @see is_right_of
%
%is_next_to(A, B, Scope) :-
    %once(is_right_of(A, B, Scope); is_left_of(A, B, Scope)).

%% is_infront_of(?Front, ?Back) is nondet.
%
% Check if Front is in front of Back. Currently does not take the orientation
% into account, only the position and dimension.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Front Identifier of the front Object
% @param Back Identifier of the back Object
%
%is_infront_of(Front, Back, Scope) :-
    %physical_object(Front),
    %map_frame_name(MapFrame),
    %object_pose(Front, [MapFrame, _, [FX,_,_], _], Time),
    %physical_object(Back),
    %Front \= Back,
    %object_pose(Back, [MapFrame, _, [BX,_,_], _], Time),
    %=<( BX, FX ).      % front obj has a higher x coord.
