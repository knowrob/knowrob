:- module(neem_logs,
    [ 
      get_actions_without_timeinterval(r),
      get_actions_with_participants_without_role(r)
      get_actions_without_tasks(r),
      get_actions_without_participants(r),
      get_child_frames_without_link_to_world_frame(r,r),
      load_logs(r)
    ]).

:- use_module(library('db/scope')).
:- use_module(library('model/DUL/Event')).
:- use_module(library('model/DUL/Object')).
:- use_module(library('ros/tf/tf_plugin')).
:- use_module(library('db/mongo/client')).


get_path(Path, Folder):-
  working_directory(X,X), string_concat(X, Folder, Path).

get_actions_without_timeinterval(ActionWithoutInterval) :-
  findall(Action, 
    ( is_action(Action),
      \+ has_time_interval(Action, TimeInterval)), 
    ActionWithoutInterval),
  length(ActionWithoutInterval, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have no time interval'),
  forall(member(Action, ActionWithoutInterval), print_message(warning, Action))), !; 
  print_message(info, 'All actions have time interval'),true.

get_actions_with_participants_without_role(ParticipantsWithoutRole) :-
  findall(Action, 
    ( is_action(Action),
      has_participant(Action, Participant), 
    \+ has_role(Participant, ParticipantRole)), 
    ParticipantsWithoutRole),
  length(ParticipantsWithoutRole, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have participants with no role. Please assert the corresponding participant role'),
  forall(member(Action, ParticipantsWithoutRole), print_message(warning, Action))), !;
  print_message(info, 'All actions have participants with a role'), true.

get_actions_without_tasks(ActionsWithNoTask) :-
  findall(Action, 
    ( is_action(Action),
    \+ executes_task(Action, Task), 
    transitive(subclass_of(Task, dul:'Task'))), 
    ActionsWithNoTask),
  length(ActionsWithNoTask, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have no tasks associated with it. Please assert the corresponding tasks performed by the action'),
  forall(member(Action, ActionsWithNoTask), print_message(warning, Action))), !;
   print_message(info, 'All actions perform a physical task'), true.


get_actions_without_participants(ActionsWithoutParticipants) :-
  findall(Action, 
    ( is_action(Action), 
    \+ has_participant(Action, P)), 
    ActionsWithoutParticipants),
  length(ActionsWithoutParticipants, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have no participants. Please assert the participants involved in the action'),
  forall(member(Action, ActionsWithoutParticipants), print_message(warning, Action))), !;
  print_message(info, 'All actions have participants'), true.


get_child_frames_without_link_to_world_frame(World, ChildFramesWithoutLinkToWorld) :-
  mng_db_name(DB),
  mng_distinct_values(DB, tf, 'child_frame_id', ChildFrames),
  findall(Frame, 
    (member(Frame, ChildFrames), 
    \+ is_at(Frame, [World, Translation, Rotation])),
    ChildFramesWithoutLinkToWorld),
  length(ChildFramesWithoutLinkToWorld, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following child frames have no link to the world frame'),
  forall(member(Child, ChildFramesWithoutLinkToWorld), print_message(warning, Child))), !;
  print_message(info, 'All the child frames in the TF are fine'), true.


load_logs(Folder) :-
  get_path(Path, Folder),
  remember(Path),
  tf_mng_remember(Path).


     /*******************************
     *          UNIT TESTS          *
     *******************************/
:- begin_tests('neem_logs').
  
test('load the logs') :-
  load_logs("neems"). % neem is the name of the folder with the logs

test('find all actions without time interval') :-
  get_actions_without_timeinterval(ActionsWithoutTimeInterval).

test('find all actions with participants assigned no role') :-
  get_actions_with_participants_without_role(ActionsWithoutRole).

test('get blackbox action which does not perform any tasks') :-
  get_actions_without_tasks(ActionsWithoutTasks).

test('get actions without participants') :-
  get_actions_without_participants(ActionsWithoutParticipants).

test('check the TF tree') :-
  WorldFrame = 'map', % Set the desired world frame 
  get_child_frames_without_link_to_world_frame(WorldFrame, ChildFrames).

:- end_tests('neem_logs').
