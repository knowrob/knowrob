
:- module(knowrob_memory,
    [
      mem_init/0,
      mem_init/1,
      mem_drop/0,
      mem_export/1,
      mem_import/1,
      mem_import_latest/0,
      mem_episode_start/1,
      mem_episode_start/2,
      mem_episode_stop/1,
      mem_episode_set_map/2,  % +Episode::iri, +Map::iri
      mem_dir/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/event_memory')).
:- use_module(library('knowrob/utility/filesystem')).

:- dynamic mem_is_initialized/0,
           current_episode/1.

%% mem_episode_start(-Episode)
%% mem_episode_start(-Episode,+Topics)
%
% Start recording an episdic memory.
%
mem_episode_start(Episode) :-
  mem_episode_start(Episode,[['tf',[]]]).

mem_episode_start(Episode,Topics) :-
  \+ current_episode(_),
  current_time(Stamp),
  %% 
  mem_episode_create(Episode),
  mem_event_begin(Episode,Stamp),
  asserta(current_episode(Episode)),
  %%
  mem_drop,
  mem_init,
  mem_create_default_index,
  mem_create_index(Topics),
  ros_logger_start(Topics).

%% mem_episode_start
%
% Stop recording episode, and export episode data to filesystem.
%
mem_episode_stop(Episode) :-
  %% get episode time extend
  current_episode(Episode),
  %%
  ros_logger_stop,
  retractall(current_episode(_)),
  %% 
  current_time(Stamp),
  mem_event_end(Episode,Stamp),
  %%
  atom_number(Stamp_atom,Stamp),
  mem_dir(MemDir),
  path_concat(MemDir,Stamp_atom,EpisodeDir),
  mem_export(EpisodeDir).

%%
mem_episode_set_map(Episode,Map) :-
  mem_event_includes_(Episode,Map).

%% mem_init
%% mem_init(+Dir)
%
% Set-up DB connection and optionally
% restore a memory dump from given directory.
%
mem_init :-
  mem_is_initialized, !.

mem_init :-
  asserta(mem_is_initialized),
  mem_db_name(DBName),
  mng_db(DBName),
  mem_create_default_index.

mem_init(Dir) :-
  mem_init,
  mem_import(Dir).

%%
mem_create_index(Topics) :-
  forall( member([Topic,IndexKeys],Topics),
          mng_create_index(Topic,IndexKeys) ).

mem_create_default_index :-
  mng_create_index(triples,
    ['begin','end','subject','sub_property']),
  mng_create_index(tf,
    ['__recorded',
     'transforms.header.stamp',
     'transforms.header.frame_id',
     'transforms.child_frame_id']).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

%% mem_export(+Dir)
%
% Dump memory into given directory.
%
mem_export(Dir) :-
  mkdir(Dir),
  exists_directory(Dir),
  %% OWL export
  path_concat(Dir,'beliefstate.owl',File),
  rdf_save(File, [graph(belief_state),sorted(true)]),
  %% Mongo DB export
  mng_dump(Dir).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

%% mem_import(+Dir)
%
% Import a memory dump.
%
mem_import(Dir) :-
  atom(Dir),
  exists_directory(Dir),!,
  print_message(informational, memory_import(Dir)),
  %% OWL import
  path_concat(Dir,'beliefstate.owl',OWLFile),
  ( exists_file(OWLFile) -> mem_import_owl(OWLFile) ; true ),
  %% Mongo DB import
  mng_restore(Dir),
  mem_import_latest_tf,
  %%
  mem_triples_init.

mem_import(OWLFile) :-
  atom(OWLFile),
  exists_file(OWLFile),
  mem_import_owl(OWLFile),
  mem_triples_init.
  

%% mem_import_latest
%
% Import latest memory dump from default directory.
%
mem_import_latest :-
  mem_dir(MemDir),
  directory_files(MemDir,Entries),
  sort(Entries,Sorted),
  reverse(Sorted,Xs),
  member(LatestName,Xs),
  atom_number(LatestName,_),!,
  path_concat(MemDir,LatestName,LatestDir),
  mem_import(LatestDir).

mem_import_owl(OWLFile) :-
  owl_parse(OWLFile, belief_state),
  %%
  belief_existing_objects(ObjectIds),
  % make sure object frames are asserted
  forall(
    member(Obj,ObjectIds),
    object_assert_frame_name(Obj,belief_state)
  ),
  % make sure object detections are stored in mongo.
  % this is to have just one central place where historical poses
  % are obtained from, and to avoid having to mix different
  % sources/queries to find out whether a pose is valid.
  forall(
    member(Obj,ObjectIds),(
    mem_import_detections(Obj),
    mem_import_fixed_pose(Obj)
  )),
  %%
  mark_dirty_objects(ObjectIds).

%%
mem_import_fixed_pose(Obj) :-
  rdf_has(Obj, knowrob:pose, Pose) -> (
    mem_pose_pl(Obj,Pose,X),
    object_pose_update(Obj,X,0)
  ) ; true.

%%
mem_import_detections(Obj) :-
  findall(Stamp-Pose, (
    object_detection(Obj,Stamp,Detection,belief_state),
    once((
      rdf_triple(knowrob:eventOccursAt,Detection,Pose),
      mem_pose_pl(Obj,Pose,[RefFrame,ObjFrame,T,Q])
    )),
    ros_store_tf([RefFrame,ObjFrame,T,Q],Stamp)
  ), StampedDetections),
  %% find latest
  ( StampedDetections=[] -> true ; (
    sort(StampedDetections,Sorted),
    reverse(Sorted,[LatestStamp-LatestPose|_]),
    object_pose_update(Obj,LatestPose,LatestStamp)
  )).

%%
mem_import_latest_tf :-
  belief_existing_objects(ObjectIds),
  forall(
    member(Obj,ObjectIds),
    ignore(mem_import_latest_tf(Obj))
  ).

mem_import_latest_tf(Obj) :-
  object_frame_name(Obj, ObjFrame),
  map_frame_name(RefFrame),
  current_time(Stamp),
  mng_lookup_transform(RefFrame,ObjFrame,pose(T,Q),Stamp),
  % FIXME use proper stamp of transform
  object_pose_update(Obj,[RefFrame,ObjFrame,T,Q],1).
  

mem_pose_pl(Obj,Pose,[ObjFrame,RefFrame,T,Q]) :-
  transform_data(Pose,(T,Q)),
  object_frame_name(Obj,ObjFrame),
  % FIXME: dulify
  (  rdf_has(Pose, knowrob:'relativeTo', Ref)
  -> object_frame_name(Ref,RefFrame)
  ;  map_frame_name(RefFrame) ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

%% mem_drop
%
% Cause amnesia -- drop all the DB tables.
%
mem_drop :-
  rdf_retractall(_, _, _, belief_state),
  forall(
    ( mng_collection(Collection),
      \+ member(Collection,['system.indexes'])
    ),
    mng_drop(Collection)).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

%%
mem_db_name(DBName) :-
  getenv('KNOWROB_MONGO_DB_NAME',DBName),
  \+ DBName='', !.
mem_db_name(roslog).

%% mem_dir(+Dir)
%
% The default directory where memory dumps are saved.
%
mem_dir(Dir) :-
  getenv('KNOWROB_MEMORY_DIR',Dir),
  \+ Dir='',
  mkdir(Dir),!.

mem_dir(Dir) :-
  getuid(UID),
  user_info(UID,Inf),
  user_data(home,Inf,Home),
  path_concat(Home,'knowrob-memory',Dir).
