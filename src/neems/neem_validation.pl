:- module(neem_validation,
    [ 
      get_actions_without_timeinterval(r),
      get_actions_with_participants_without_role(r),
      get_actions_without_tasks(r),
      get_actions_without_participants(r),
      %get_child_frames_without_link_to_world_frame(r,r),
      get_objects_without_location(r),
      get_objects_without_shape(r),
      get_joints_without_proper_links(r),
      first_n_list(r,r,r),
      load_logs(r),
      validate_episode(r)
    ]).

:- use_module(library('mongodb/client')).
:- use_module(library('scope')).
:- use_module(library('model/DUL')).
:- use_module(library('model/SOMA')).
:- use_module(library('ros/tf/tf')).
:- use_module(library('ros/urdf/URDF')).


get_path(Path, Folder):-
  ros_package_path('knowrob', X),
  atom_concat(X, '/', Temp), 
  atom_concat(Temp, Folder, Path).

get_actions_without_timeinterval(ActionWithoutInterval) :-
  findall(Action, 
    ( is_action(Action),
      \+ has_time_interval(Action, _)), 
    ActionWithoutInterval),
  length(ActionWithoutInterval, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have no time interval'),
  first_n_list(10, ActionWithoutInterval, NActionWithoutInterval),
  forall(member(Action, NActionWithoutInterval), print_message(warning, Action))), !; 
  print_message(info, 'All actions have time intervals'),true.

get_actions_with_participants_without_role(ParticipantsWithoutRole) :-
  findall(Action, 
    ( 
      ask(aggregate([
        triple(Action,rdf:type,dul:'Event'),
        triple(Action,rdf:type,regex('^.*(?!Action).*')),
        triple(Action,dul:hasParticipant,Participant)
      ])),
      \+ has_role(Participant, _)
    ), 
  Participants),
  list_to_set(Participants, ParticipantsWithoutRole),
  length(ParticipantsWithoutRole, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have participants with no role. Please assert the corresponding participant role'),
  first_n_list(10, ParticipantsWithoutRole, NParticipantsWithoutRole),
  forall(member(Action, NParticipantsWithoutRole), print_message(warning, Action))), !;
  print_message(info, 'All actions have participants with a role'), true.

get_actions_without_tasks(ActionsWithNoTask) :-
  findall(Action, 
    ( is_action(Action),
    \+ executes_task(Action, Task), 
    transitive(subclass_of(Task, dul:'Task'))), 
    ActionsWithNoTask),
  length(ActionsWithNoTask, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have no tasks associated with it. Please assert the corresponding tasks performed by the action'),
  first_n_list(10, ActionsWithNoTask, NActionsWithNoTask),
  forall(member(Action, NActionsWithNoTask), print_message(warning, Action))), !;
  print_message(info, 'All actions perform a physical task'), true.


get_actions_without_participants(ActionsWithoutParticipants) :-
  findall(Action, 
    ( is_action(Action), 
    \+ has_participant(Action, _)), 
    ActionsWithoutParticipants),
  length(ActionsWithoutParticipants, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following actions have no participants. Please assert the participants involved in the action'),
  first_n_list(10, ActionsWithoutParticipants, NActionsWithoutParticipants),
  forall(member(Action, NActionsWithoutParticipants), print_message(warning, Action))), !;
  print_message(info, 'All actions have participants'), true.


%get_child_frames_without_link_to_world_frame(World, ChildFramesWithoutLinkToWorld) :-
%  mng_db_name(DB),
%  mng_distinct_values(DB, tf, 'child_frame_id', ChildFrames),
%  findall(Frame, 
%    (member(Frame, ChildFrames), 
%    \+ is_at(Frame, [World, _, _])),
%    ChildFramesWithoutLinkToWorld),
%  length(ChildFramesWithoutLinkToWorld, Listlength),
%  ( Listlength > 0 -> print_message(warning, 'The following child frames have no link to the world frame'),
%  forall(member(Child, ChildFramesWithoutLinkToWorld), print_message(warning, Child))), !;
%  print_message(info, 'All the child frames in the TF are fine'), true.

get_objects_without_location(POWithoutLocation) :-
   findall(Object,
    ( is_physical_object(Object),
    \+ object_localization(Object, _)),
    POWithoutLocation),
  length(POWithoutLocation, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following physical objects have no location. Please assert the locations for these objects'),
  first_n_list(10, POWithoutLocation, NPOWithoutLocation),
  forall(member(O, NPOWithoutLocation), print_message(warning, O))), !;
  print_message(info, 'All objects have a location'), true.

get_objects_without_shape(POWithoutShape) :-
   findall(Object,
    ( is_physical_object(Object),
    (\+ object_shape(Object, _, Shape, Pos, Material); \+ ros_urdf:object_shape(Object, _, Shape, Pos, Material))),
    PO),
  list_to_set(PO, POWithoutShape),
  length(POWithoutShape, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following physical objects have no shape. Please assert the shape for these objects or load the corresponding urdf files for the robot and environment, where the shapes are defined.'),
  first_n_list(10, POWithoutShape, NPOWithoutShape),
  forall(member(O, NPOWithoutShape), print_message(warning, O))), !;
  print_message(info, 'All physical objects in this episode have shape'), true.

get_joints_without_proper_links(Joints) :-
  findall(Joint,
    ( has_type(Joint, urdf:'Joint'),
    \+ has_parent_link(Joint, _),
    \+ has_child_link(Joint, _)),
    Joints),
  length(Joints, Listlength),
  ( Listlength > 0 -> print_message(warning, 'The following joints have no parent link or child link asserted. Please assert the corresponding links for the joints'),
  first_n_list(10, Joints, NJoints),
  forall(member(O, NJoints), print_message(warning, O))), !;
  print_message(info, 'All joints have their corresponding links'), true.

first_n_list(N, List, Front):- length(Front, N), append(Front, _, List).

load_logs(Folder) :-
  get_path(Path, Folder),
  remember(Path),
  tf_mng_remember(Path).

validate_episode(_WorldFrame):- % Set the name of the folder with the logs as Foldername, the desired world frame
  get_actions_without_timeinterval(_),
  get_actions_with_participants_without_role(_),
  get_actions_without_tasks(_),
  get_actions_without_participants(_),
  %get_child_frames_without_link_to_world_frame(WorldFrame, _),
  get_objects_without_location(_),
  get_objects_without_shape(_),
  get_joints_without_proper_links(_).
