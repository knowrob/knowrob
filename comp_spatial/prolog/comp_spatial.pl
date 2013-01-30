
/** <module> comp_spatial

  This module contains all computables that calculate qualitative spatial relations
  between objects to allow for spatial reasoning. In addition, there are computables
  to extract components of a matrix or position vector.


  Copyright (C) 2010 by Moritz Tenorth, Lars Kunze

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth, Lars Kunze
@license GPL
*/

:- module(comp_spatial,
    [
      holds/2,
      on_Physical/2,
      in_ContGeneric/2,
      comp_toTheRightOf/2,
      comp_toTheLeftOf/2,
      comp_toTheSideOf/2,
      comp_inFrontOf/2,
      comp_inCenterOf/2,
      comp_below_of/2,
      comp_above_of/2,
      comp_center/2,
      objectAtPoint2D/2,
      objectAtPoint2D/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('knowrob_objects')).

:- owl_parser:owl_parse('../owl/comp_spatial.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_spatial, 'http://ias.cs.tum.edu/kb/comp_spatial.owl#', [keep(true)]).



% define holds as meta-predicate and allow the definitions
% to be in different parts of the source file
:- meta_predicate holds(0, ?, ?).
:- discontiguous holds/2.

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    holds(:, r),
    on_Physical(r, r),
    in_ContGeneric(r, r),
    adjacent_Objects(r, r),
    comp_below_of(r,r),
    comp_above_of(r,r),
    comp_toTheSideOf(r, r),    comp_toTheRightOf(r, r),    comp_toTheLeftOf(r, r),
    comp_inFrontOf(r, r),
    comp_inCenterOf(r, r),
    comp_center(r, r).




%% on_Physical(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%
on_Physical(Top, Bottom) :-
    get_timepoint(T),
    holds(on_Physical(Top, Bottom), T).



%% holds(+OnPhysicalPred:compound, +T) is nondet.
%
% Usage: holds(on_Physical(?Top, ?Bottom), +T)
%
% Check if Top has been in the area of and above Bottom at time point T.
%
% Currently does not take the orientation into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%

%%% holds(on_Physical(Top, Bottom), T) :-
%%%
%%%     object_detection(Top, T, VPT),
%%%     object_detection(Bottom, T, VPB),
%%%
%%%     rdf_triple(knowrob:eventOccursAt, VPT,    TopMatrix),
%%%     rdf_triple(knowrob:eventOccursAt, VPB, BottomMatrix),
%%%
%%%     rdf_triple(knowrob:m03, TopMatrix, literal(type(_,TCx))),atom_to_term(TCx,TX,_),
%%%     rdf_triple(knowrob:m13, TopMatrix, literal(type(_,TCy))),atom_to_term(TCy,TY,_),
%%%     rdf_triple(knowrob:m23, TopMatrix, literal(type(_,TCz))),atom_to_term(TCz,TZ,_),
%%%
%%%     rdf_triple(knowrob:m03, BottomMatrix, literal(type(_,BCx))),atom_to_term(BCx,BX,_),
%%%     rdf_triple(knowrob:m13, BottomMatrix, literal(type(_,BCy))),atom_to_term(BCy,BY,_),
%%%     rdf_triple(knowrob:m23, BottomMatrix, literal(type(_,BCz))),atom_to_term(BCz,BZ,_),
%%%
%%%     % read the dimensions of the bottom entity
%%%     rdf_triple(knowrob:widthOfObject, Bottom, literal(type(_,Bw))),atom_to_term(Bw,BW,_),
%%%     rdf_triple(knowrob:depthOfObject, Bottom, literal(type(_,Bd))),atom_to_term(Bd,BD,_),
%%%
%%%     % the criterion is if the difference between them is less than epsilon=5cm
%%%     =<( BZ, TZ ),
%%%
%%%     % additional criterion: center of the top entity has to be inside the
%%%     % area of the bottom entity
%%%     =<( (BX - 0.5*BD), TX ), >=( (BX + 0.5*BD), TX ),
%%%     =<( (BY - 0.5*BW), TY ), >=( (BY + 0.5*BW), TY ),
%%%     Top \= Bottom.
holds(on_Physical(Top, Bottom),T) :-

    % get object center for Top
    object_detection(Top, T, VPT),
    rdf_triple(knowrob:eventOccursAt, VPT,    TopMatrix),
    rdf_triple(knowrob:m03, TopMatrix, TCxx),strip_literal_type(TCxx, TCx),atom_to_term(TCx,TX,_),
    rdf_triple(knowrob:m13, TopMatrix, TCyy),strip_literal_type(TCyy, TCy),atom_to_term(TCy,TY,_),
    rdf_triple(knowrob:m23, TopMatrix, TCzz),strip_literal_type(TCzz, TCz),atom_to_term(TCz,TZ,_),

%     rdf_triple(knowrob:heightOfObject, Top, literal(type(_,Th))),atom_to_term(Th,TH,_),

    % query for objects at center point
    objectAtPoint2D(TX,TY,Bottom),

    % get height of objects at center point
    object_detection(Bottom, T, VPB),
    rdf_triple(knowrob:eventOccursAt, VPB, BottomMatrix),
    rdf_triple(knowrob:m23, BottomMatrix, BCzz), strip_literal_type(BCzz, BCz),atom_to_term(BCz,BZ,_),
%     rdf_triple(knowrob:heightOfObject, Bottom, literal(type(_,Bh))),atom_to_term(Bh,BH,_),

%     print('bottom height:'), print(BH),

    % the criterion is if the difference between them is less than epsilon=5cm
    <( BZ, TZ),
    Top \= Bottom.





%% comp_above_of(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%
comp_above_of(Top, Bottom) :-
    get_timepoint(T),
    holds(comp_below_of(Bottom, Top), T).



%% comp_below_of(?Bottom, ?Top) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Bottom Identifier of the lower Object
% @param Top Identifier of the upper Object
%
comp_below_of(Bottom, Top) :-
    get_timepoint(T),
    holds(comp_below_of(Bottom, Top), T).



%% holds(+BelowOf:compound, +T) is nondet.
%
% Usage: holds(comp_below_of(?Top, ?Bottom), +T)
%
% Check if Bottom has been in the area of and below Top at time point T.
%
% Currently does not take the orientation into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%

holds(comp_below_of(Bottom, Top),T) :-

    % get object center for Bottom
    object_detection(Bottom, T, VPB),

    rdf_triple(knowrob:eventOccursAt, VPB,    BottomMatrix),
    rdf_triple(knowrob:m03, BottomMatrix, BCxx),strip_literal_type(BCxx, BCx),atom_to_term(BCx,BX,_),
    rdf_triple(knowrob:m13, BottomMatrix, BCyy),strip_literal_type(BCyy, BCy),atom_to_term(BCy,BY,_),
    rdf_triple(knowrob:m23, BottomMatrix, BCzz),strip_literal_type(BCzz, BCz),atom_to_term(BCz,BZ,_),

    % query for objects at center point
    objectAtPoint2D(BX,BY,Top),

    % get objects at center point
    object_detection(Top, T, VPT),
    rdf_triple(knowrob:eventOccursAt, VPT, TopMatrix),
    rdf_triple(knowrob:m23, TopMatrix, TCzz), strip_literal_type(TCzz, TCz),atom_to_term(TCz,TZ,_),

    >( TZ, BZ ),
    Top \= Bottom.


%% comp_toTheLeftOf(?Left, ?Right) is nondet.
%
% Check if Left is to the left of Right.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Left Identifier of the left Object
% @param Right Identifier of the right Object
%
comp_toTheLeftOf(Left, Right) :-
    get_timepoint(T),
    holds(comp_toTheLeftOf(Left, Right), T).


%% holds(+ToTheLeftOf:compound, +T) is nondet.
%
% Usage: holds(comp_toTheLeftOf(?Left, ?Right), +T)
%
% Check if Left is to the left of Right. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%
holds(comp_toTheLeftOf(Left, Right), T) :-
    %
    % TODO: adapt this to take rotations and object dimensions into account
    %

    object_detection(Left, T, VPL),
    object_detection(Right, T, VPR),

    rdf_triple(knowrob:eventOccursAt, VPL, LeftMatrix),
    rdf_triple(knowrob:eventOccursAt, VPR, RightMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, LeftMatrix, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
    rdf_triple(knowrob:m13, LeftMatrix, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
    rdf_triple(knowrob:m23, LeftMatrix, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, RightMatrix, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
    rdf_triple(knowrob:m13, RightMatrix, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
    rdf_triple(knowrob:m23, RightMatrix, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),

    =<( abs( LX - RX), 0.30),  % less than 30cm y diff
    =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
    =<( abs( LZ - RZ), 0.30),  % less than 30cm height diff
    Left \= Right.



%% comp_toTheRightOf(?Right,?Left) is nondet.
%
% Check if Right is to the right of Left.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @see comp_toTheLeftOf
%
comp_toTheRightOf(Right, Left) :-
    get_timepoint(T),
    holds(comp_toTheRightOf(Right, Left), T).


%% holds(+ToTheRightOf:compound, +T) is nondet.
%
% Usage: holds(comp_toTheRightOf(?Right,?Left), +T)
%
% Check if Right is to the right of Left.
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @param T      TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
%
holds(comp_toTheRightOf(Right, Left), T) :-
    holds(comp_toTheLeftOf(Left, Right), T).




%% comp_toTheSideOf(?A, ?B) is nondet.
%
% Check if A is either to the left or the rigth of B.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
%
comp_toTheSideOf(A, B) :-
    get_timepoint(T),
    holds(comp_toTheSideOf(A, B), T).

%% holds(+ToTheLeftOf:compound, +T) is nondet.
%
% Usage: holds(comp_toTheSideOf(?A, ?B), +T) is nondet.
%
% Check if A is either to the left or the right of B.
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @param T TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
%
holds(comp_toTheSideOf(A, B), T) :-
    holds(comp_toTheRightOf(A, B), T);
    holds(comp_toTheLeftOf(A, B), T).




%% comp_inFrontOf(?Front, ?Back) is nondet.
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
comp_inFrontOf(Front, Back) :-
    get_timepoint(T),
    holds(comp_inFrontOf(Front, Back), T).

%% holds(+InFrontOf:compound, +T) is nondet.
%
% Usage: holds(comp_inFrontOf(?Front, ?Back), +T)
%
% Check if Front is in front of Back. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Front Identifier of the front Object
% @param Back Identifier of the back Object
% @param T TimePoint or Event for which the relations is supposed to hold
%
holds(comp_inFrontOf(Front, Back), T) :-
    %
    % TODO: adapt this to take rotations and object dimensions into account
    %

    object_detection(Front, T, VPF),
    object_detection(Back, T, VPB),

    rdf_triple(knowrob:eventOccursAt, VPF, FrontMatrix),
    rdf_triple(knowrob:eventOccursAt, VPB, BackMatrix),

    % read the center coordinates of the front entity
    rdf_triple(knowrob:m03, FrontMatrix, literal(type(_,FCx))),atom_to_term(FCx,FX,_),

    % read the center coordinates of the back entity
    rdf_triple(knowrob:m03, BackMatrix, literal(type(_,BCx))),atom_to_term(BCx,BX,_),

    =<( BX, FX ),      % front obj has a higher x coord
    Front \= Back.




%% comp_inCenterOf(?Inner, ?Outer) is nondet.
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
comp_inCenterOf(Inner, Outer) :-
    get_timepoint(T),
    holds(comp_inCenterOf(Inner, Outer), T).

%% holds(+InCenterOf:compound, +T) is nondet.
%
% Usage: holds(comp_inCenterOf(?Inner, ?Outer), +T)
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
% @param T TimePoint or Event for which the relations is supposed to hold
%
holds(comp_inCenterOf(Inner, Outer), T) :-

    object_detection(Inner, T, VPI),
    object_detection(Outer, T, VPO),

    rdf_triple(knowrob:eventOccursAt, VPI, InnerMatrix),
    rdf_triple(knowrob:eventOccursAt, VPO, OuterMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, InnerMatrix, literal(type(_,ICx))),atom_to_term(ICx,IX,_),
    rdf_triple(knowrob:m13, InnerMatrix, literal(type(_,ICy))),atom_to_term(ICy,IY,_),
    rdf_triple(knowrob:m23, InnerMatrix, literal(type(_,ICz))),atom_to_term(ICz,IZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, OuterMatrix, literal(type(_,OCx))),atom_to_term(OCx,OX,_),
    rdf_triple(knowrob:m13, OuterMatrix, literal(type(_,OCy))),atom_to_term(OCy,OY,_),
    rdf_triple(knowrob:m23, OuterMatrix, literal(type(_,OCz))),atom_to_term(OCz,OZ,_),

    =<( abs( IX - OX), 0.20),  % less than 20cm x diff
    =<( abs( IY - OY), 0.20),  % less than 20cm y diff
    =<( abs( IZ - OZ), 0.20),  % less than 20cm z diff
    Inner \= Outer.


%% in_ContGeneric(?InnerObj, ?OuterObj) is nondet.
%
% Check if InnerObj is contained by OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
                                % Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
%
in_ContGeneric(InnerObj, OuterObj) :-
    get_timepoint(T),
    holds(in_ContGeneric(InnerObj, OuterObj), T).


%% holds(+InContained:compound, +T) is nondet.
%
% Usage: holds(in_ContGeneric(?InnerObj, ?OuterObj), +T)
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
% @param T TimePoint or Event for which the relations is supposed to hold
%


% MT: tried to use matrix transformation library to perform easier computation of 'inside'
% using bounding box. Problem; does not work as long as not both objects are bound
%
% holds(in_ContGeneric(InnerObj, OuterObj), T) :-
%
%
% % TODO: take time into account
%
%     nonvar(InnerObj), nonvar(OuterObj),
%     transform_relative_to(InnerObj, OuterObj, [_,_,_,IrelOX,_,_,_,IrelOY,_,_,_,IrelOZ,_,_,_,_]),
%
%     % read the dimensions of the outer entity
%     rdf_triple(knowrob:widthOfObject, OuterObj, LOW),strip_literal_type(LOW,Ow),atom_to_term(Ow,OW,_),
%     rdf_triple(knowrob:heightOfObject,OuterObj, LOH),strip_literal_type(LOH,Oh),atom_to_term(Oh,OH,_),
%     rdf_triple(knowrob:depthOfObject, OuterObj, LOD),strip_literal_type(LOD,Od),atom_to_term(Od,OD,_),
%
%
%     % is InnerInOuterCoordList inside bounding box of outer object?
%
%     >=( OD, IrelOX),
%     >=( OW, IrelOY),
%     >=( OH, IrelOZ),
%
%     InnerObj \= OuterObj.

holds(in_ContGeneric(InnerObj, OuterObj), T) :-

    (var(InnerObj); var(OuterObj)),

    object_detection(InnerObj, T, VPI),
    object_detection(OuterObj, T, VPO),

    rdf_triple(knowrob:eventOccursAt, VPI, InnerObjMatrix),
    rdf_triple(knowrob:eventOccursAt, VPO, OuterObjMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, InnerObjMatrix, LICX),strip_literal_type(LICX,ICx),atom_to_term(ICx,IX,_),
    rdf_triple(knowrob:m13, InnerObjMatrix, LICY),strip_literal_type(LICY,ICy),atom_to_term(ICy,IY,_),
    rdf_triple(knowrob:m23, InnerObjMatrix, LICZ),strip_literal_type(LICZ,ICz),atom_to_term(ICz,IZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, OuterObjMatrix, LOCX),strip_literal_type(LOCX,OCx),atom_to_term(OCx,OX,_),
    rdf_triple(knowrob:m13, OuterObjMatrix, LOCY),strip_literal_type(LOCY,OCy),atom_to_term(OCy,OY,_),
    rdf_triple(knowrob:m23, OuterObjMatrix, LOCZ),strip_literal_type(LOCZ,OCz),atom_to_term(OCz,OZ,_),

    % read the dimensions of the outer entity
    rdf_triple(knowrob:widthOfObject, OuterObj, LOW),strip_literal_type(LOW,Ow),atom_to_term(Ow,OW,_),
    rdf_triple(knowrob:heightOfObject,OuterObj, LOH),strip_literal_type(LOH,Oh),atom_to_term(Oh,OH,_),
    rdf_triple(knowrob:depthOfObject, OuterObj, LOD),strip_literal_type(LOD,Od),atom_to_term(Od,OD,_),

    % read the dimensions of the inner entity
    rdf_triple(knowrob:widthOfObject, InnerObj, LIW),strip_literal_type(LIW,Iw),atom_to_term(Iw,IW,_),
    rdf_triple(knowrob:heightOfObject,InnerObj, LIH),strip_literal_type(LIH,Ih),atom_to_term(Ih,IH,_),
    rdf_triple(knowrob:depthOfObject, InnerObj, LID),strip_literal_type(LID,Id),atom_to_term(Id,ID,_),

    % InnerObj is contained by OuterObj if (center_i+0.5*dim_i)<=(center_o+0.5*dim_o)
    % for all dimensions (x, y, z)
    >=( (IX - 0.5*ID), (OX - 0.5*OD)-0.05), =<( (IX + 0.5*ID),  (OX + 0.5*OD)+0.05 ),
    >=( (IY - 0.5*IW), (OY - 0.5*OW)-0.05 ), =<( (IY + 0.5*IW), (OY + 0.5*OW)+0.05 ),
    >=( (IZ - 0.5*IH), (OZ - 0.5*OH)-0.05 ), =<( (IZ + 0.5*IH), (OZ + 0.5*OH)+0.05 ),
    InnerObj \= OuterObj.





% % % % % % % % % % % % % % % % % % % %
% matrix and vector computations (relating the homography-based
% position representation with the old center-point-based one)
%

%% comp_center(+Obj, ?Center) is semidet.
%
% Compute the center point of an object from its homography matrix
%
% @param Obj    The object identifier
% @param Center The center point identifier as a String 'translation_<rotation matrix identifier>'
    comp_center(Obj, Center) :-
      rdf_triple(knowrob:orientation, Obj, Matrix),
      rdf_split_url(G, L, Matrix),
      atom_concat('translation_', L, P),
      rdf_split_url(G, P, Center).



% todo: generalize projection to floor. polygon instead of rectangle.
objectAtPoint2D(Point2D, Obj) :-
    % get coordinates of point of interest
    rdf_triple(knowrob:xCoord, Point2D, PCxx), strip_literal_type(PCxx, PCx), atom_to_term(PCx,PX,_),
    rdf_triple(knowrob:yCoord, Point2D, PCyy), strip_literal_type(PCyy, PCy), atom_to_term(PCy,PY,_),
    objectAtPoint2D(PX,PY,Obj).

%
% FIXME: THIS IS BROKEN FOR ALL NON-STANDARD ROTATIONS if the upper left matrix is partly zero
%

objectAtPoint2D(PX,PY,Obj) :-

    % get information of potential objects at positon point2d (x/y)

    rdf_triple(knowrob:depthOfObject, Obj, Oww), strip_literal_type(Oww, Ow),atom_to_term(Ow,OW,_),
    rdf_triple(knowrob:widthOfObject, Obj, Odd), strip_literal_type(Odd, Od),atom_to_term(Od,OD,_),

    rdf_triple(knowrob:orientation, Obj, Mat),
    rdf_triple(knowrob:m03, Mat, Tmm03), strip_literal_type(Tmm03, TM03),atom_to_term(TM03,OX,_),
    rdf_triple(knowrob:m13, Mat, Tmm13), strip_literal_type(Tmm13, TM13),atom_to_term(TM13,OY,_),
    rdf_triple(knowrob:m00, Mat, Tmm00), strip_literal_type(Tmm00, TM00),atom_to_term(TM00,M00,_),
    rdf_triple(knowrob:m01, Mat, Tmm01), strip_literal_type(Tmm01, TM01),atom_to_term(TM01,M01,_),
    rdf_triple(knowrob:m10, Mat, Tmm10), strip_literal_type(Tmm10, TM10),atom_to_term(TM10,M10,_),
    rdf_triple(knowrob:m11, Mat, Tmm11), strip_literal_type(Tmm11, TM11),atom_to_term(TM11,M11,_),

    % object must have an extension
    <(0,OW), <(0,OD),

    % calc corner points of rectangle (consider rectangular objects only!)
    P0X is (OX - 0.5*OW),
    P0Y is (OY + 0.5*OD),
    P1X is (OX + 0.5*OW),
    P1Y is (OY + 0.5*OD),
    P2X is (OX - 0.5*OW),
    P2Y is (OY - 0.5*OD),
    % rotate points
    RP0X is (OX + (P0X - OX) * M00 + (P0Y - OY) * M01),
    RP0Y is (OY + (P0X - OX) * M10 + (P0Y - OY) * M11),
    RP1X is (OX + (P1X - OX) * M00 + (P1Y - OY) * M01),
    RP1Y is (OY + (P1X - OX) * M10 + (P1Y - OY) * M11),
    RP2X is (OX + (P2X - OX) * M00 + (P2Y - OY) * M01),
    RP2Y is (OY + (P2X - OX) * M10 + (P2Y - OY) * M11),

    % debug: print rotated points
    %write('P0 X: '), write(P0X), write(' -> '), writeln(RP0X),
    %write('P0 Y: '), write(P0Y), write(' -> '), writeln(RP0Y),
    %write('P1 X: '), write(P1X), write(' -> '), writeln(RP1X),
    %write('P1 Y: '), write(P1Y), write(' -> '), writeln(RP1Y),
    %write('P2 X: '), write(P2X), write(' -> '), writeln(RP2X),
    %write('P2 Y: '), write(P2Y), write(' -> '), writeln(RP2Y),

    V1X is (RP1X - RP0X),
    V1Y is (RP1Y - RP0Y),

    V2X is (RP2X - RP0X),
    V2Y is (RP2Y - RP0Y),

    VPX is (PX - RP0X),
    VPY is (PY - RP0Y),

    DOT1 is (VPX * V1X + VPY * V1Y),
    DOT2 is (VPX * V2X + VPY * V2Y),
    DOTV1 is (V1X * V1X + V1Y * V1Y),
    DOTV2 is (V2X * V2X + V2Y * V2Y),

    =<(0,DOT1), =<(DOT1, DOTV1),
    =<(0,DOT2), =<(DOT2, DOTV2).


% compatibility with Prolog < 5.8
:- if(\+current_predicate(atomic_list_concat, _)).

  atomic_list_concat(List, Atom) :-
    concat_atom(List, Atom).

  atomic_list_concat(List, Separator, Atom) :-
    concat_atom(List, Separator, Atom).

:- endif.
