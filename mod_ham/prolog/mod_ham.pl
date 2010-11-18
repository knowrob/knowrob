/** <module> mod_ham

  Description:
    Recursively abstract sequences of action seqments into more and more
    high-level representations.


  Copyright (C) 2010 by Moritz Tenorth

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

@author Moritz Tenorth
@license GPL
*/

:- module(mod_ham,
    [
      compute_abstraction_hierarchy/0,
      compute_and_visualize_abstraction_hierarchy/1,
      compute_and_visualize_abstraction_hierarchy_cmu/1,

      perform_action_abstraction/3,
      perform_and_visualize_action_abstraction/6,
      perform_and_visualize_action_abstraction_cmu/6,

      leftHandInfos/1,
      rightHandInfos/1,
      next_abstracted_action_layer/6,
      condense_label_sequence/7,
      propagate_object_acted_on/1,
      print_action_sequence/1,
      action_properties/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('comp_fipm')).
:- use_module(library('comp_temporal')).
:- use_module(library('comp_spatial')).
:- use_module(library('mod_vis')).
:- use_module(library('util')).

:- initCmdProlog.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% High-level predicates for calling the system from outside
%


%% compute_abstraction_hierarchy is nondet.
%
% Top-level predicate: retrieves action data for both hands, computes the hierarchical action
% models, and displays the result in the console
% 
compute_abstraction_hierarchy :-

  % load motion data, calculate features, classify frames using CRF
  motions_loaded(LFrames, LLabels, LTimes, 'left'),

  % create action segments from the classified frame data, determine action data (object, hand, etc)
  motions_to_actions(LFrames, LLabels, LTimes, 'florian', 'left', 0, LActions),

  % recursively compute more and more abstract representations of the actions
  perform_action_abstraction(LActions, _, 100),

  % ... and the same for the right hand
  motions_loaded(RFrames, RLabels, RTimes, 'right'),
  motions_to_actions(RFrames, RLabels, RTimes, 'florian', 'right', 50, RActions),
  perform_action_abstraction(RActions, _, 150).



%% compute_and_visualize_action_abstraction(-Canvas) is nondet.
%
% Top-level predicate: retrieves action data for both hands, computes the hierarchical action
% models, and displays the result in the visualisation canvas
%
% Like compute_abstraction_hierarchy, but displays the result graphically in a visualization canvas
%
% @param Canvas An instance of the VisualizationCanvas (see mod_vis.pl) that the
%               result of the action interpretation procedure is displayed in.
%
compute_and_visualize_abstraction_hierarchy(Canvas) :-

  % launch visualisation module
  visualisation_canvas(Canvas),

  % load motion data, calculate features, classify frames using CRF
  motions_loaded(LFrames, LLabels, LTimes, 'left'),

  % create action segments from the classified frame data, determine action data (object, hand, etc)
  motions_to_actions(LFrames, LLabels, LTimes, 'florian', 'left', 0, LActions),

  % recursively compute more and more abstract representations of the actions
  perform_and_visualize_action_abstraction(LActions, _, 100, 0, 'left', Canvas),

  % ... and the same for the right hand
  motions_loaded(RFrames, RLabels, RTimes, 'right'),
  motions_to_actions(RFrames, RLabels, RTimes, 'florian', 'right', 50, RActions),
  perform_and_visualize_action_abstraction(RActions, _, 150, 0, 'right', Canvas).



%% perform_action_abstraction(+LowLevelActions, -HighLevelActions, +Index) is nondet.
%
% Build the abstraction pyramid for one sequence of labels by recursive abstraction
%
% @params LowLevelActions   Sequence of action instances that is to be abstracted
% @params HighLevelActions  Resulting most-abstract action sequence
% @params Index             Start index that is used for generating object instances called <class>_<index>
%
perform_action_abstraction(LowLevelActions, HighLevelActions, Index) :-

  print('\n\n\n'), print('Action sequence: '), print_action_sequence(LowLevelActions), print('\n\n'),

  print('Performing abstraction step...'), print('\n'),
  next_abstracted_action_layer(LowLevelActions, [], [], Index, [], AbstractedActions1),
  flatten(AbstractedActions1, AbstractedActions),
  ((lists_equal(LowLevelActions, AbstractedActions)) ->
    (HighLevelActions=AbstractedActions, print('Abstraction pyramid converged.'), print('\n')) ;
    (Index1 is Index+40, perform_action_abstraction(AbstractedActions, HighLevelActions, Index1))).




%% perform_and_visualize_action_abstraction(+LowLevelActions, -HighLevelActions, +Index, +Hand, +Canvas) is nondet.
%
% Build the abstraction pyramid for one sequence of labels by recursive abstraction
%
% Like perform_action_abstraction, but displays the result graphically in a visualization canvas
% 
% @params LowLevelActions   Sequence of action instances that is to be abstracted
% @params HighLevelActions  Resulting most-abstract action sequence
% @params Index             Start index that is used for generating object instances called <class>_<index>
% @params Hand
% @params Canvas
%
perform_and_visualize_action_abstraction(LowLevelActions, HighLevelActions, Index, Level, Hand, Canvas) :-

  % print initial action sequence
  print('\n\n\nAction sequence: '), print_action_sequence(LowLevelActions), print('\n\n'),

  % determine action properties (start/end time, object, hand, ...) for each action and display the result
  findall(Info, (member(A, LowLevelActions), action_properties(A, Info)), Infos),
  lists_to_arrays(Infos, SeqJava),
  show_actionseq(SeqJava, Canvas, Hand,  Level),

  % try to compute a more abstract action representation
  print('Performing abstraction step...'), print('\n'),
  next_abstracted_action_layer(LowLevelActions, [], [], Index, [], AbstractedActions1),
  flatten(AbstractedActions1, AbstractedActions),

  % compare the result with the previous list of actions and continue
  % if an improvement could be achieved
  ((lists_equal(LowLevelActions, AbstractedActions)) ->
    (HighLevelActions=AbstractedActions, print('Abstraction pyramid converged.'), print('\n')) ;
    (Index1 is Index+40, Level1 is Level+1, perform_and_visualize_action_abstraction(AbstractedActions, HighLevelActions, Index1, Level1, Hand, Canvas))).


%
% special version for working on the CMU data
%
% TODO: fuse this with the other predicates to allow for ONE general abstraction mechanism
%

%% compute_and_visualize_abstraction_hierarchy_cmu(-Canvas) is nondet.
% 
% see above
%
compute_and_visualize_abstraction_hierarchy_cmu(Canvas) :-

  visualisation_canvas(Canvas),

  findall(A, rdfs_instance_of(A, knowrob:'Action'), Actions),
  predsort(actionCompare, Actions, ActionsSorted),

  abstraction_hierarchy_cmu(ActionsSorted, _, 100, 0, 'left',Canvas).



%% perform_and_visualize_action_abstraction_cmu(+LowLevelActions, -HighLevelActions, +Index, +Level, +Hand, +Canvas) is nondet.
%
% see above
% 
perform_and_visualize_action_abstraction_cmu(LowLevelActions, _HighLevelActions, Index, Level, Hand, Canvas) :-

  % print initial action sequence
  print('\n\n\nAction sequence: '), print_action_sequence(LowLevelActions), print('\n\n'),

  % determine action properties (start/end time, object, hand, ...) for each action and display the result
  findall(Info, (member(A, LowLevelActions), action_properties(A, Info)), Infos),
  lists_to_arrays(Infos, SeqJava),
  show_actionseq(SeqJava, Canvas, Hand,  Level),

  % iteration 1
  print('Performing abstraction step...'), print('\n'),
  next_abstracted_action_layer(LowLevelActions, [], [], Index, [], AbstractedActions1),
  flatten(AbstractedActions1, AbstractedActions),

  print('\n\n\nAction sequence: '), print_action_sequence(AbstractedActions), print('\n\n'),
  findall(AInfo, (member(A, AbstractedActions), action_properties(A, AInfo)), AInfos),
  lists_to_arrays(AInfos, ASeqJava),
  Level1 is Level+1,
  show_actionseq(ASeqJava, Canvas, Hand,  Level1),

  % iteration 2
  print('Performing abstraction step...'), print('\n'),
  Index1 is Index+30,
  next_abstracted_action_layer(AbstractedActions, [], [], Index1, [], AbstractedActions2),
  flatten(AbstractedActions2, AbstractedActions3),

  print('\n\n\nAction sequence: '), print_action_sequence(AbstractedActions3), print('\n\n'),
  findall(A3Info, (member(A3, AbstractedActions3), action_properties(A3, A3Info)), A3Infos),
  lists_to_arrays(A3Infos, A3SeqJava),
  Level2 is Level1+1,
  show_actionseq(A3SeqJava, Canvas, Hand,  Level2).










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Performing the abstraction steps
%




%% next_abstracted_action_layer(+LowLevelActions, +SuperActHypotheses, +ActionSegm, +Index, -HighLevelActions) is nondet.
%
% Perform one abstraction step
%
% @params LowLevelActions     Sequence of action instances that is to be abstracted
% @params SuperActHypotheses  (Initially empty) set of possible actions that match the current segment so far; the predicate will check which of the hypotheses fit the current action as well.
% @params ActionSegm          List of action instances that are part of the current action segment, i.e. which are supposed to belong to the same super-action
% @params Index               Start index that is used for generating object instances called <class>_<index>
% @params HighLevelActions    Resulting abstracted action sequence
% 
next_abstracted_action_layer([A|LowLevelActions], _, ActionSegm, Index, OtherActionSequences, [ActionSegm1|HighLevelActions]) :-
  % if there are no possible super-actions at all, simply keep the action and continue with the next segment
  \+ compatible_super_actions(A, _, []),
  append(ActionSegm, A, ActionSegm1),
  !,next_abstracted_action_layer(LowLevelActions, [], [], Index, OtherActionSequences, HighLevelActions).

next_abstracted_action_layer([A|LowLevelActions], SuperActHypotheses, ActionSegm, Index, OtherActionSequences, HighLevelActions) :-
  % continue with the segment if
  % - the current action fits to at least one of the super-actions
  (   setof(Super, compatible_super_actions(A, Super, SuperActHypotheses), CompatibleHypotheses),

%   % - the object did not change
%       check_object_acted_on(ActionSegm, A),

  % - the ordering constraints are met so far
      append(ActionSegm, [A], ActionSegm1),

      (SuperActHypotheses=[]; (member(H, CompatibleHypotheses),
      findall(C, ordering_constraints(H, C), OrderingConstraints),
      check_ordering_constraints_soft(ActionSegm1, OrderingConstraints))),

  % - and there is no super-action yet
      \+ (rdf_has(_, knowrob:subEvents, A))

  ) -> (

      (
        (not(OrderingConstraints=[]),check_ordering_constraints_strict(ActionSegm1, OrderingConstraints)) -> (

          % segment complete
          !, create_superevents([H], ActionSegm1, Index, Index1, HI),
          next_abstracted_action_layer(LowLevelActions, [], [], Index1, OtherActionSequences, HighLevelActions1),
          append(HI, HighLevelActions1, HighLevelActions)

        );(
          % continue with next action in the segment
          !, next_abstracted_action_layer(LowLevelActions, CompatibleHypotheses, ActionSegm1, Index, OtherActionSequences, HighLevelActions)
        )
      )

  ) ; (

    !, 

    % or: branch off to the alternative sequences
    multi_sequence_action(CompatibleHypotheses, ActionSegm1, Index, OtherActionSequences, HighLevelActions),


    % otherwise finish the segment and continue with a new high-level action
    create_superevents(SuperActHypotheses, ActionSegm, Index, Index1, HI),

    ((setof(Super, compatible_super_actions(A, Super, []), NewHypotheses)) ->
      next_abstracted_action_layer(LowLevelActions, NewHypotheses, [A], Index1, OtherActionSequences, HighLevelActions1) ;
      next_abstracted_action_layer(LowLevelActions, [],            [A], Index1, OtherActionSequences, HighLevelActions1) ),
    append(HI, HighLevelActions1, HighLevelActions)
    ).

% finish the last segment if needed
next_abstracted_action_layer([], SuperActHypotheses, ActionSegm, Index, _, HI) :-

  \+ length(SuperActHypotheses, 0),
  \+ length(ActionSegm, 0),
  create_superevents(SuperActHypotheses, ActionSegm, Index, _, HI).



multi_sequence_action(CompatibleHypotheses, ActionSegm, Index, OtherActionSequences, HighLevelActions) :-

  member(OtherActionSeq, OtherActionSequences),
  member(A, OtherActionSeq),
  rdf_has(A, knowrob:startTime, StT),

  % startTime > endTime
  


  % only the first three items after that?

  % ordering constraints are met so far
  append(ActionSegm, [A], ActionSegm1),

  (SuperActHypotheses=[]; (member(H, CompatibleHypotheses),
  findall(C, ordering_constraints(H, C), OrderingConstraints),
  check_ordering_constraints_soft(ActionSegm1, OrderingConstraints))),

  % same object?

  create_superevents(SuperActHypotheses, ActionSegm, Index, _, HighLevelActions).


%% create_superevents(CompatibleHypotheses, ActionSegm, Index, Index1, HIlist) is nondet.
%
% Iterates over the set of super-action hypotheses, checks if all sub-action and
% ordering constraints are met and, in this case, asserts this super-action including
% its properties like the objectActedOn or the startTime. The resulting list of
% super-action instances is returned in HIlist.
%
% If some of the constraints are not met, the sub-actions are simply copied to HIlist,
% meaning that they did not have sensible super-actions and thus should appear unchanged
% on the higher level of abstraction.
%
% @param CompatibleHypotheses List of super-action hypotheses that are supposed to match the segment
% @param ActionSegm           Segment of actions that are supposed to belong to a common super-action
% @param Index                Start index that is used for generating object instances called <class>_<index>
% @param Index1               Incremented index (returned to higher-level predicates to avoid equally named instances)
% @param HIlist               Resulting set of action instances at the higher level of abstraction
% 
create_superevents(CompatibleHypotheses, ActionSegm, Index, Index1, HIlist) :-

    member(H, CompatibleHypotheses),!,

    % check if all constraints are met and assert the actions themselves otherwise
    (findall(C, ordering_constraints(H, C), OrderingConstraints),
      check_ordering_constraints_strict(ActionSegm, OrderingConstraints)) ->
      (
        % create instances of the hypotheses
        atom_concat(H, Index, HI),
        Index1 is Index+1,
        rdf_assert(HI, rdf:type, H),
        HIlist=[HI],

        % subevents
        findall(Act, (member(Act, ActionSegm),rdf_assert(HI, knowrob:subEvents, Act)), _Acts),

        % objActedOn
        findall(Act, (member(Act, ActionSegm),
                        findall(Obj, (rdf_triple(knowrob:objectActedOn, Act, Obj)), Objs),
                        findall(O,   (member(O, Objs), rdf_assert(HI, knowrob:objectActedOn, O)), _)
                        ), _),

        % doneBy
        findall(Act, (member(Act, ActionSegm),
                        findall(Actor, (rdf_triple(knowrob:doneBy, Act, Actor)), Actors),
                        findall(Actr,  (member(Actr, Actors), rdf_assert(HI, knowrob:doneBy, Actr)), _)
                        ), _),


        % bodyPartUsed
        findall(Act, (member(Act, ActionSegm),
                        findall(Part, (rdf_triple(knowrob:bodyPartsUsed, Act, Part)), Parts),
                        findall(P,    (member(P, Parts), rdf_assert(HI, knowrob:bodyPartsUsed, P)), _)
                        ), _),

        % fromLocation
        findall(Act, (member(Act, ActionSegm),
                        findall(FLoc, (rdf_triple(knowrob:fromLocation, Act, FLoc)), FLocs),
                        findall(FL,   (member(FL, FLocs), rdf_assert(HI, knowrob:fromLocation, FL)), _)
                        ), _),

        % toLocation
        findall(Act, (member(Act, ActionSegm),
                        findall(TLoc, (rdf_triple(knowrob:toLocation, Act, TLoc)), TLocs),
                        findall(TL,   (member(TL, TLocs), rdf_assert(HI, knowrob:toLocation, TL)), _)
                        ), _),


        % startTime
        findall(St, (member(Act, ActionSegm),
                     rdf_triple(knowrob:startTime, Act, StString),
                     ( (concat_atom(SLst, '#', StString),length(SLst, 1)) % check if the time is given including the URL

                         ->(term_to_atom(St, StString))

                         ; (concat_atom(SLst, '#', StString),
                            nth0(1, SLst, StLocal),
                            atom_concat('timepoint_', StAtom, StLocal),
                            term_to_atom(St, StAtom) ))
                     ), Sts),
        util:min_list(Sts, StartTime),term_to_atom(StartTime, StartTimeAtom),
        atom_concat('timepoint_', StartTimeAtom, StartTimeLocal),
        rdf_global_id(knowrob:StartTimeLocal, StartTimeRdf),
        rdf_assert(HI, knowrob:startTime, StartTimeRdf),

        % endTime
        findall(Et,(member(Act, ActionSegm),
                     rdf_triple(knowrob:endTime, Act, EtString),
                     ( (concat_atom(ELst, '#', EtString),length(ELst, 1)) % check if the time is given including the URL

                         ->(term_to_atom(Et, EtString))

                         ; (concat_atom(ELst, '#', EtString),
                            nth0(1, ELst, EtLocal),
                            atom_concat('timepoint_', EtAtom, EtLocal),
                            term_to_atom(Et, EtAtom) ))
                     ), Ets),
        util:max_list(Ets, EndTime),term_to_atom(EndTime, EndTimeAtom),
        atom_concat('timepoint_', EndTimeAtom, EndTimeLocal),
        rdf_global_id(knowrob:EndTimeLocal, EndTimeRdf),
        rdf_assert(HI, knowrob:endTime, EndTimeRdf)

      ) ; (
        HIlist=ActionSegm, Index1=Index
      ).





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Reading and processing actions and action-related information
%

%% motions_loaded(-Frames, -Labels, -Times, +Hand) is nondet.
%
% Read motions from DB, compute features, classify with CRF
%
motions_loaded(Frames, Labels, Times, 'left') :-
  leftHandPosesLabeled(Frames, Labels, Times).
motions_loaded(Frames, Labels, Times, 'right') :-
  rightHandPosesLabeled(Frames, Labels, Times).


%% motions_to_actions(+Frames, +Labels, +Times, +Actor, +Hand, +StartIndex, -Actions) is nondet.
%
% Create action segments based on the classified frame data, and determine (and assert)
% action properties like the objectActedOn.
%
motions_to_actions(Frames, Labels, Times, Actor, Hand, StartIndex, Actions) :-
  condense_label_sequence(Labels, Frames, Times, Actor, Hand, Actions, StartIndex),
  propagate_object_acted_on(Actions).


%% leftHandInfos(-SeqJava) is nondet.
%% rightHandInfos(-SeqJava) is nondet.
%
% Creates a sequence of actions by performing one abstraction step on the CRF output
% and determining the action parameters, transforminng this action into a Java object
%
% @param SeqJava Result transformed to a Java object
%
leftHandInfos(SeqJava) :-
    leftHandPosesLabeled(Frames, Labels, Times),
    condense_label_sequence(Labels, Frames, Times, 'florian', 'left', Actions, 100),
    propagate_object_acted_on(Actions),
    next_abstracted_action_layer(Actions, [], [], 140, [], HLA),
    findall(Info, (member(A, HLA), action_properties(A, Info)), Infos),
    lists_to_arrays(Infos, SeqJava).

rightHandInfos(SeqJava) :-
    rightHandPosesLabeled(Frames, Labels, Times),
    condense_label_sequence(Labels, Frames, Times, 'florian', 'right', Actions, 0),
    propagate_object_acted_on(Actions),
    next_abstracted_action_layer(Actions, [], [], 40, [], HLA),
    findall(Info, (member(A, HLA), action_properties(A, Info)), Infos),
    lists_to_arrays(Infos, SeqJava).


%% condense_label_sequence(+Labels, +Frames, +Times, +Actor, +Hand, -Actions, +Index) is nondet.
%
% Assert single frames as OWL instances, add actor and object information, and
% aggregate all subsequent, identical labels to one action instance
%
% @param Labels   List of labels, e.g. the output of a CRF classifier
% @param Frames   List of frame identifiers that belong to the labels
% @param Times    List of time stamps belonging to the Frames
% @param Actor    Agent who performed the actions in Frames
% @param Hand     The hand used by the Agent
% @param Actions  Resulting list of actions, generated by aggregating all subsequent frames with the same label to one action instance of the resp. type
% @param Index    Start index that is used for generating object instances called <class>_<index>
%
condense_label_sequence([L|Labels], [F|Frames], [T|Times], Actor, Hand, [AI|Actions], Index) :-

  % create new super-event
  label_mapping(L, A),
  atom_concat(A, Index, AI),
  rdf_assert(AI, rdf:type, A),
  Index1 is Index+1,

  % assert start time
  atom_concat('timepoint_', T, Ttimest),
  rdf_global_id(knowrob:Ttimest, Trdf),
  rdf_assert(Trdf, rdf:type, knowrob:'TimePoint'),
  rdf_assert(AI, knowrob:startTime, Trdf),

  % iterate over frames until the action changes
  condense_label_sequence_1([L|Labels], [F|Frames], [T|Times], L, AI, Actor, Hand, LabelRest, FrameRest, TimesRest, LastTimeInSegm),!,

  % assert end time
  atom_concat('timepoint_', LastTimeInSegm, LastTimeTimest),
  rdf_global_id(knowrob:LastTimeTimest, LastTimeRdf),
  rdf_assert(LastTimeRdf, rdf:type, knowrob:'TimePoint'),
  rdf_assert(AI, knowrob:endTime, LastTimeRdf),

  % assert the doneBy and bodyPartsUsed relations
  rdf_assert(AI, knowrob:doneBy, Actor),
  rdf_assert(AI, knowrob:bodyPartsUsed, Hand),

  % iterate further on the higher level (re-using the result list of the lower level)
  condense_label_sequence(LabelRest, FrameRest, TimesRest, Actor, Hand, Actions, Index1),

  % add the next-pointers between actions and frames
   ((nth0(0, Actions, NxtAI),nonvar(NxtAI)) ->
     rdf_assert(AI, knowrob:nextAction, NxtAI)
   ; (true)).

condense_label_sequence([], [], [], _, _, [], _).


% helper predicate for condense_label_sequence
condense_label_sequence_1([L|Labels], [F|Frames], [T|Times], L, AI, Actor, Hand, LabelRest, FrameRest, TimesRest, LastTimeInSegm) :-

  nth0(0, Labels, L),
  % create RDF representation for the current frame
  concat_atom(F, ')-(', F1),atom_concat('Pose-(', F1, F2), atom_concat(F2, ')', FI),
  rdf_assert(FI, rdf:type, knowrob:'Posture-Configuration'),

  % assert the startTime relation and create a new timestamp
  atom_concat('timepoint_', T, Ttimest),
  rdf_global_id(knowrob:Ttimest, Trdf),
  rdf_assert(Trdf, rdf:type, knowrob:'TimePoint'),
  rdf_assert(FI, knowrob:startTime, Trdf),

  % assert the doneBy and bodyPartsUsed relations
  rdf_assert(FI, knowrob:doneBy, Actor),
  rdf_assert(FI, knowrob:bodyPartsUsed, Hand),

  % link the current frame to the super-event
  rdf_assert(AI, knowrob:subEvents, FI),

  % continue
  condense_label_sequence_1(Labels, Frames, Times, L, AI, Actor, Hand, LabelRest, FrameRest, TimesRest, LastTimeInSegm).

% helper predicate for condense_label_sequence
% return when the next label is not equal to the previous one
condense_label_sequence_1([L|Labels], [F|Frames], [T|Times], L, AI, Actor, Hand, Labels, Frames, Times, T) :-

  not(nth0(0, Labels, L)),
  % create RDF representation for the current frame
  concat_atom(F, ')-(', F1),atom_concat('Pose-(', F1, F2), atom_concat(F2, ')', FI),
  rdf_assert(FI, rdf:type, knowrob:'Posture-Configuration'),

  % assert the startTime relation and create a new timestamp
  atom_concat('timepoint_', T, Ttimest),
  rdf_global_id(knowrob:Ttimest, Trdf),
  rdf_assert(Trdf, rdf:type, knowrob:'TimePoint'),
  rdf_assert(FI, knowrob:startTime, T),

  % assert the doneBy and bodyPartsUsed relations
  rdf_assert(Actor, rdf:type, knowrob:'Agent'),
  rdf_assert(Hand,  rdf:type, knowrob:'AnimalBodyPart'),
  rdf_assert(FI, knowrob:doneBy, Actor),
  rdf_assert(FI, knowrob:bodyPartsUsed, Hand),

  % link the current frame to the super-event
  rdf_assert(AI, knowrob:subEvents, FI).



%% propagate_object_acted_on(+Actions) is nondet.
%
% Propagate objectActedOn relations to neighboring reach/take/put/release segments
%
% This becomes necessary if the objectActedOn is determined e.g. by RFID tag readers. In that
% case, the information from the instantaneous RFID event falls into only one of the action
% segments (e.g. TakingSomething), but the Reaching segment just before should have the same
% objectActedOn.
%
% @params Actions An action sequence where not all of the actions have the objectActedOn property set
%
propagate_object_acted_on([First|Actions]) :-
  nth0(0, Actions, Next),
  \+ (
     (rdf_has(First, rdf:type, knowrob:'Reaching'), rdf_has(Next, rdf:type, knowrob:'TakingSomething')) ;
     (rdf_has(First, rdf:type, knowrob:'LoweringAnObject'), rdf_has(Next, rdf:type, knowrob:'ReleasingGraspOfSomething'))
  ),
  !,propagate_object_acted_on(Actions).


propagate_object_acted_on([First|Actions]) :-

  nth0(0, Actions, Next),
  (rdf_has(First, rdf:type, knowrob:'Reaching'), rdf_has(Next, rdf:type, knowrob:'TakingSomething')),!,

  % assert the objectActedOn for the missing one in the pair
  ( (rdf_triple(knowrob:objectActedOn, First, Obj),
     \+ rdf_triple(knowrob:objectActedOn, Next, Obj))
     ->
    (rdf_assert(Next, knowrob:objectActedOn, Obj))
    ; (true)),
  ( (\+ rdf_triple(knowrob:objectActedOn, First, Obj),
        rdf_triple(knowrob:objectActedOn, Next, Obj))
     ->
    (!,rdf_assert(First, knowrob:objectActedOn, Obj))
    ; (true)),
    !,propagate_object_acted_on(Actions).


propagate_object_acted_on([First|Actions]) :-

  nth0(0, Actions, Next),
  (rdf_has(First, rdf:type, knowrob:'LoweringAnObject'), rdf_has(Next, rdf:type, knowrob:'ReleasingGraspOfSomething')),!,

  % assert the objectActedOn for the missing one in the pair
  ( (rdf_triple(knowrob:objectActedOn, First, Obj),
     \+ rdf_triple(knowrob:objectActedOn, Next, Obj))
     ->
    (rdf_assert(Next, knowrob:objectActedOn, Obj))
    ; (true)),
  ( (\+ rdf_triple(knowrob:objectActedOn, First, Obj),
        rdf_triple(knowrob:objectActedOn, Next, Obj))
     ->
    (!,rdf_assert(First, knowrob:objectActedOn, Obj))
    ; (true)),
    !,propagate_object_acted_on(Actions).

propagate_object_acted_on([_]).
propagate_object_acted_on([]).



%% action_properties(+Frame, -ActionInformation) is nondet.
%
% Read action information and create something like the Java ActionInformation class
%
% @param Frame              Action instance identifier
% @param ActionInformation  Result: [[Frame], [StartTime], [EndTime], Types, [Objecttype]]
%
action_properties(Frame, ActionInformation) :-

  ( (rdf_triple(knowrob:startTime, Frame, StT))
      -> (concat_atom(StList, '#', StT), nth0(1, StList, StLocal),
          atom_concat('timepoint_', StAtom, StLocal),
          term_to_atom(StartTime, StAtom) ) ;
         (StartTime='')),
  ( (rdf_triple(knowrob:endTime, Frame, EnT))
      -> (concat_atom(EtList, '#', EnT), nth0(1, EtList, EtLocal),
          atom_concat('timepoint_', EtAtom, EtLocal),
          term_to_atom(EndTime, EtAtom) ) ;
         (EndTime='')),

  setof(T, rdf_has(Frame, rdf:type, T), Types),
  ((rdf_triple(knowrob:objectActedOn, Frame, Objecttype),!);Objecttype=''),
  ActionInformation=[[Frame], [StartTime], [EndTime], Types, [Objecttype]].


%% ordering_constraints(+Super, -Constr)
%
% read ordering constraints on subEvents of an action
%
% @param Super
% @param Constr
%
ordering_constraints(Super, Constr) :-
  rdf_has(Restr, owl:hasValue, Constr),
  rdf_has(Restr, owl:onProperty, knowrob:orderingConstraints),
  rdf_has(Super, rdfs:subClassOf, Restr).


%% super_event_type(?Super, ?Sub)
%
% Read all possible super-events of the current action type
%
% @param Super
% @param Sub
%
super_event_type(Super, Sub) :-
  rdf_has(Restr, owl:someValuesFrom, Sub),
  rdf_has(Restr, owl:onProperty, knowrob:subEvents),
  rdf_has(Super, rdfs:subClassOf, Restr).



%% compatible_super_actions(+Sub, +SuperT, +Hypotheses)
%
% check if super event is compatible with the given list of hypotheses, unless that list is empty
%
% @param Sub
% @param SuperT
% @param Hypotheses
%
compatible_super_actions(Sub, SuperT, Hypotheses) :-
  rdf_has(Sub, rdf:type, SubT),
  super_event_type(SuperT, SubT),
  member(SuperT, Hypotheses).
compatible_super_actions(Sub, SuperT, []) :-
  rdf_has(Sub, rdf:type, SubT),
  super_event_type(SuperT, SubT).





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Checking action properties
%


%% check_ordering_constraints_soft(+ActionSegm, +Constraints) is nondet.
%
% check if at least some of the ordering constraints are met
%
% @param ActionSegm   Segment of action instances
% @param Constraints  Constraints to be checked on this segment
% 
check_ordering_constraints_soft(_, []).
check_ordering_constraints_soft([_], _).

check_ordering_constraints_soft(ActionSegm, Constraints) :-
  % check if there is no constraint that is NOT satisfied
  length(ActionSegm, L), L>=2,

  \+ (
    member(C, Constraints),
    rdf_has(C, knowrob:occursBeforeInOrdering, C1),
    rdf_has(C, knowrob:occursAfterInOrdering,  C2),

    member(A1, ActionSegm), rdf_has(A1, rdf:type, C1),
    member(A2, ActionSegm), rdf_has(A2, rdf:type, C2),

    nth0(I1, ActionSegm, A1),
    nth0(I2, ActionSegm, A2),

    (I1 > I2)
  ).



%% check_ordering_constraints_strict(+ActionSegm, +Constraints) is nondet.
%
% Verify that all ordering constraints are met by the current action segment
%
% @param ActionSegm   Segment of action instances
% @param Constraints  Constraints to be checked on this segment
% 
check_ordering_constraints_strict(_, []).
check_ordering_constraints_strict(ActionSegm, [C|Constraints]) :-

  length(ActionSegm, L), L>=2,

  rdf_has(C, knowrob:occursBeforeInOrdering, C1),
  rdf_has(C, knowrob:occursAfterInOrdering,  C2),
  member(A1, ActionSegm), rdf_has(A1, rdf:type, C1),
  member(A2, ActionSegm), rdf_has(A2, rdf:type, C2),

  forall(
    (   member(A1, ActionSegm), rdf_has(A1, rdf:type, C1),
        member(A2, ActionSegm), rdf_has(A2, rdf:type, C2),
        nth0(I1, ActionSegm, A1),
        nth0(I2, ActionSegm, A2)
    ), (I1 =< I2)),
  check_ordering_constraints_strict(ActionSegm, Constraints).



%% check_object_acted_on(+ActionSegm, +Constraints) is nondet.
%
% Verify that all actions in ActionSegm have the same objectActedOn as Action
% @param ActionSegm List of action instances
% @param Action     Action instance
% 
check_object_acted_on([], _).
check_object_acted_on([A|ActionSegm], Action) :-

  % objectActedOn matches or no objectActedOn asserted
  (rdf_triple(knowrob:objectActedOn, Action, O); \+ rdf_triple(knowrob:objectActedOn, Action, _)),!,
  (rdf_triple(knowrob:objectActedOn, A, O); \+ rdf_triple(knowrob:objectActedOn, A, _)),!,

  check_object_acted_on(ActionSegm, Action).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Helper functions: pretty printing, mappings etc.
%

%% print_action_sequence(+Actions) is nondet.
%
% Print a nice description of the action sequence (omit namespace etc.)
%
% @param Actions The sequence of action instances to be printed to the console
%
print_action_sequence([]).
print_action_sequence([First]) :-
  rdf_split_url(_, Local, First),
  print(Local).
print_action_sequence([First|Rest]) :-
  rdf_split_url(_, Local, First),
  print(Local),print(', '),
  print_action_sequence(Rest).


%% label_mapping(+Label, -Class)
%
% Map from the informal labels to classes in the knowledge base
%
% @param Label Informal label used in the TUM kitchen data set
% @param Class Corresponding class in the KnowRob ontology
% 
label_mapping('idle_carry',     'http://ias.cs.tum.edu/kb/knowrob.owl#CarryingWhileLocomoting').
label_mapping('reach',          'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching').
label_mapping('reach_up',       'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching').
label_mapping('take',           'http://ias.cs.tum.edu/kb/knowrob.owl#TakingSomething').
label_mapping('take_up',        'http://ias.cs.tum.edu/kb/knowrob.owl#TakingSomething').
label_mapping('put',            'http://ias.cs.tum.edu/kb/knowrob.owl#LoweringAnObject').
label_mapping('release',        'http://ias.cs.tum.edu/kb/knowrob.owl#ReleasingGraspOfSomething').
label_mapping('release_up',     'http://ias.cs.tum.edu/kb/knowrob.owl#ReleasingGraspOfSomething').
label_mapping('open_cupboard',  'http://ias.cs.tum.edu/kb/knowrob.owl#OpeningADoor').
label_mapping('close_cupboard', 'http://ias.cs.tum.edu/kb/knowrob.owl#ClosingADoor').
label_mapping('open_drawer',    'http://ias.cs.tum.edu/kb/knowrob.owl#OpeningADrawer').
label_mapping('close_drawer',   'http://ias.cs.tum.edu/kb/knowrob.owl#ClosingADrawer').
label_mapping('standing',       'http://ias.cs.tum.edu/kb/knowrob.owl#StandingStill').
label_mapping('moving',         'http://ias.cs.tum.edu/kb/knowrob.owl#HumanWalkingProcess').


%
% predicates for sorting actions based on their instance number
% 
actionCompare(Delta, A1, A2) :-
    rdf_split_url(_, L1str, A1),
    rdf_split_url(_, L2str, A2),
    string_to_list(L1str, L1),
    string_to_list(L2str, L2),
    actionNr(L1, N1),
    actionNr(L2, N2),
    compare(Delta, N1, N2).

actionNr(ActId, ActNr) :-
    string_tokens(ActId, TokenList),
    last(TokenList, ActNrList),
    string_to_list(ActNrStr, ActNrList),
    atom_to_term(ActNrStr, ActNr, _).