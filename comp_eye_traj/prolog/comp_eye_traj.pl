/** <module> comp_eye_traj

  This module contains all computables for eye trajectories

  Copyright (C) 2011 by Karinne Ramirez-Amaro

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

@author Karinne Ramirez-Amaro
@license GPL
*/

:-module(comp_eye_traj, 
	[
	 objectInAction/3,
	 readEyeTrajectory/5,
         objectInHand/6,
         objectInHand/3,
         objectRecognized/5,
         objRecBeforeAction/5,
	 actionObjectRec/4
	]).

:- owl_parser:owl_parse('../owl/comp_eye_traj.owl',  false, false, true).


:- rdf_db:rdf_register_ns(knowrob,   'http://ias.cs.tum.edu/kb/knowrob.owl#',        [keep(true)]).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicate for Detect the object acted on while performing an activity
%
% Example: Type= 'http://ias.cs.tum.edu/kb/knowrob.owl#onObject'
% Action= 'http://ias.cs.tum.edu/kb/knowrob.owl#PickingUpAnObject'

objectInAction(Type, Action, Obj) :-
    rdf_triple(Type, Action, O), 
    rdf_split_url(_, Obj, O).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicate for identify and highligh the object that is the hand of the person
%
% Action ='http://ias.cs.tum.edu/kb/knowrob.owl#Reaching'
% Hand= 'http://ias.cs.tum.edu/kb/knowrob.owl#RightHand'
% Canvas = $C
% Obj= is the object we want to highligh

objectInHand(Action,Hand, Ai, ObjInhand, ObjActed, Canvas) :-
	rdfs_instance_of(Ainst, Action),
	rdf_split_url(_, Ai, Ainst),
	rdf_triple(knowrob:'bodyPartsUsed', Ainst, Hand), 
	rdf_triple(knowrob:'onObject', Ainst, Obj),
	rdf_split_url(_, ObjInhand, Obj),
	rdf_triple(knowrob:'objectActedOn', Ainst, ObjA),
	rdf_split_url(_, ObjActed, ObjA),
	owl_has(Oinst, rdf:type, Obj), 
	highlight_object(Oinst, @(true),-65530, Canvas),  %Red color
	owl_has(OAins, rdf:type, ObjA),
	highlight_object(OAins, @(true),-256, Canvas).  %object manipulated

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicate for identify the action the 
% person that is performing while having certain object
% or also gives the object that is in the hand of the person if you give the 
% appropiate instance of the Action
%
% Action ='http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_1'
% Hand= 'http://ias.cs.tum.edu/kb/knowrob.owl#RightHand'
% ObjIn= is the object we want or 'http://ias.cs.tum.edu/kb/knowrob.owl#Spatula'

objectInHand(Action,Hand, ObjIn) :-
	rdf_triple(knowrob:'bodyPartsUsed', Action, Hand), 
	rdf_triple(knowrob:'onObject', Action, ObjIn).

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Query: While manipulating the pancake mix, what objects do the person look at and what 
% actions was he/she performing?
%
% ObjActedOn: 'http://ias.cs.tum.edu/kb/knowrob.owl#PancakeMix'

actionObjectRec(Action, ObjActedOn, ObjRec, Canvas):-
	rdf_triple(knowrob:'objectActedOn', Act, ObjectActedOn), 
	rdf_split_url(_, Action, Act),
	rdf_triple(knowrob:'detectedObject', Act, ObjR),
	rdf_split_url(_, ObjRec, ObjR),
	owl_has(Orec, rdf:type, ObjR),
	highlight_object(Orec, @(true), -65530, Canvas).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicate for identify theo object that is being recognized by the 
% gaze camera, while performing an specific activity 
% We can answer the query: Where was the eye looking at when the person was reaching?
%
% Action ='http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_1'
% Obj= is the object we want to highligh  or 'http://ias.cs.tum.edu/kb/knowrob.owl#Spatula'
% Canvas = $C

objectRecognized(Action, Acti, ObjActedOn, ObjRec, Canvas) :-
	rdfs_instance_of(Ainst, Action),
	rdf_split_url(_, Acti, Ainst),
	rdf_triple(knowrob:'objectActedOn', Ainst, ObjA),
	rdf_split_url(_, ObjActedOn, ObjA),
	owl_has(OAOinst, rdf:type, ObjA),
	rdf_triple(knowrob:'detectedObject', Ainst, ObjR),
	rdf_split_url(_, ObjRec, ObjR),
	owl_has(Oinst, rdf:type, ObjR),
	highlight_object(OAOinst, @(true), -256, Canvas), %yellow object
	highlight_object(Oinst, @(true),-16766465, Canvas). %blue object

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicate for identify the object that is recogniced before reaching an specific action
% Query to answer this question: What object did the person look before reaching?
%
% Action ='http://ias.cs.tum.edu/kb/knowrob.owl#Reaching'


objRecBeforeAction(Action, Act, ObjectRec, ObjActedOn, Canvas) :-
	rdfs_instance_of(Ainst, Action),
	rdf_split_url(_,Act, Ainst),
	rdf_triple(knowrob:'before', Ainst, Obj),
	rdf_split_url(_,ObjectRec, Obj),
	owl_has(Oinst, rdf:type, Obj),
        highlight_object(Oinst, @(true), -65299, Canvas), %pink object
        rdf_triple(knowrob:'objectActedOn', Ainst, ObjA),
	rdf_split_url(_,ObjActedOn, ObjA),
 	owl_has(Oaoi, rdf:type, ObjA),
        highlight_object(Oaoi, @(true), -256, Canvas).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Predicates for reading eye trajectories 
%
%for example Type = 'http://ias.cs.tum.edu/kb/knowrob.owl#PickingUpAnObject'
% T=trajectoryId, P=PointID, XCoor and YCoor are the X and Y coordinates

readEyeTrajectory(Type, T, P, XCoor, YCoor) :-
%    rdf_triple(rdf:type, A, Type), 
    rdfs_instance_of(A, Type),
    rdf_triple(knowrob:'trajectory-Eye', A, Traj),
    rdf_split_url(_, T, Traj),
    setof(Point, (rdf_triple(knowrob:pointOnEyeTrajectory, Traj, Pnt),rdf_split_url(_, Point, Pnt)), Ps),
    predsort(trajPointCompare, Ps, Psorted),
    member(Pt, Psorted),
    term_to_atom(P-XCoor-YCoor, Pt).

readEyeTrajectory(A, T, P, FIRX, FIRY, FIRZ, FILX, FILY, FILZ) :- 
    rdf_triple(knowrob:'trajectory-Arm', A, T),
    setof(Point, (rdf_triple(knowrob:pointOnArmTrajectory, T, Pnt),rdf_split_url(_, Point, Pnt)), Ps),
    predsort(trajPointCompare, Ps, Psorted),
    member(Pt, Psorted),
    term_to_atom(P-FIRX-FIRY-FIRZ-FILX-FILY-FILZ, Pt).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Utilities
%

% util: comparison function for trajectory points (required for sorting)
trajPointCompare(Delta, P1, P2) :- 
    term_to_atom(S1-_-_-_-_-_-_, P1),
    term_to_atom(S2-_-_-_-_-_-_, P2),
    string_to_list(S1, L1),
    string_to_list(S2, L2),
    trajPointInstanceNr(L1, N1),
    trajPointInstanceNr(L2, N2),
    compare(Delta, N1, N2).


% util: transform point id of form P_1_2_33 into 33 (extract instance number)
trajPointInstanceNr(PointId, InstNr) :-
    string_tokens(PointId, TokenList),
    last(TokenList, InstNrList),
    string_to_list(InstNrStr, InstNrList),
    atom_to_term(InstNrStr, InstNr, _).




