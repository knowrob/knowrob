:- module(neem_validation,
    [ 
      get_actions_without_timeinterval(r),
      get_actions_with_participants_without_role(r),
      get_actions_without_tasks(r),
      get_actions_without_participants(r),
      get_child_frames_without_link_to_world_frame(r,r),
      get_objects_without_location(r),
      get_objects_without_shape(r),
      get_joints_without_proper_links(r),
      load_logs(r),
      validate_episode(r,r)
    ]).

:- use_module(library('db/scope')).
:- use_module(library('model/DUL/Event')).
:- use_module(library('model/DUL/Object')).
:- use_module(library('ros/tf/tf_plugin')).
:- use_module(library('db/mongo/client')).
:- use_module(library('ros/urdf/URDF')).


get_path(Path, Folder):-
  ros_package_path('knowrob', X),
  atom_concat(X, '/', Temp), 
  atom_concat(Temp, Folder, Path).

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
    Participants),
  list_to_set(Participants, ParticipantsWithoutRole),
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

get_objects_without_location(POWithoutLocation) :-
   findall(Object,
    ( is_physical_object(Object),
    \+ object_localization(Object, Location)),
    POWithoutLocation),
  length(POWithoutLocation, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following physical objects have no location. Please assert the locations for these objects'),
  forall(member(O, POWithoutLocation), print_message(warning, O))), !;
   print_message(info, 'All objects have a location'), true.

get_objects_without_shape(POWithoutShape) :-
   findall(Object,
    ( is_physical_object(Object),
    (\+ object_shape(Object, Shape, Pos, Material); \+ ros_urdf:object_shape(Object, Shape, Pos, Material))),
    PO),
  list_to_set(PO, POWithoutShape),
  length(POWithoutShape, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following physical objects have no shape. Please assert the shape for these objects or load the corresponding urdf files for the robot and environment, where the shapes are defined.'),
  forall(member(O, POWithoutShape), print_message(warning, O))), !;
   print_message(info, 'All physical objects in this episode have shape'), true.

get_joints_without_proper_links(Joints) :-
  findall(Joint,
    ( has_type(Joint, urdf:'Joint'),
    \+ has_parent_link(Joint, _),
    \+ has_child_link(Joint, _)),
    Joints),
  length(Joints, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following joints have no parent link or child link asserted. Please assert the corresponding links for the joints'),
  forall(member(O, Joints), print_message(warning, O))), !;
  print_message(info, 'All joints have their corresponding links'), true.

load_logs(Folder) :-
  get_path(Path, Folder),
  remember(Path),
  tf_mng_remember(Path).

validate_episode(Foldername, WorldFrame):- % Set the name of the folder with the logs as Foldername, the desired world frame. Neems folder is expected to be in knowrob package
  load_logs(Foldername),
  get_actions_without_timeinterval(ActionsWithoutTimeInterval),
  get_actions_with_participants_without_role(ActionsWithoutRole),
  get_actions_without_tasks(ActionsWithoutTasks),
  get_actions_without_participants(ActionsWithoutParticipants),
  get_child_frames_without_link_to_world_frame(WorldFrame, ChildFrames),
  get_objects_without_location(ObjectWithoutLocation),
  get_objects_without_shape(ObjWithoutShape),
  get_joints_without_proper_links(JointWithoutPLinkOrCLink).



     /*******************************
     *          UNIT TESTS          *
     *******************************/
:- begin_tests('neem_validation').

test('validate the stored episode') :-
  validate_episode('neems', 'map').

:- end_tests('neem_validation').
