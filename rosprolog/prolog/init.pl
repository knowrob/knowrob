/*
  Copyright (C) 2009-14 Lorenz Mösenlechner, Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Lorenz Mösenlechner, Moritz Tenorth
@license BSD

*/

:- use_module(library('process')).

%% ros_param_get_string(+Key,-Value) is semidet
%% ros_param_get_double(+Key,-Value) is semidet
%% ros_param_get_int(+Key,-Value) is semidet
%
% Read parameter from ROS parameter server.
%%
%% ros_param_set_string(+Key,+Value) is semidet
%% ros_param_set_double(+Key+Value) is semidet
%% ros_param_set_int(+Key,+Value) is semidet
%
% Write parameter to ROS parameter server.
%%
%% ros_info(+Msg) is det
%% ros_warning(+Msg) is det
%% ros_error(+Msg) is det
%
% Debug via ROS master.
%%
%% ros_package_path(+Package, -Path) is semidet
%
% Locate ROS packages on the harddisk using 'rospack find'
%%
%% ros_package_command(+Command, -Output) is semidet
%
% Runs a rospack command of the form 'rospack ...'
%%
%% tf_lookup_transform(+TargetFrame, +SourceFrame, -Transform) is semidet
%
% True if Transform is the current transform from TargetFrame to SourceFrame.
% Transform is a term "pose([X,Y,Z],[QX,QY,QZ,QW])".
%%
%% tf_listener_start is det
% 
% Start listening to TF topic.
%%
%% tf_transform_point(+SourceFrame, +TargetFrame, +PointIn, -PointOut) is semidet
%% tf_transform_quaternion(+SourceFrame, +TargetFrame, +QuaternionIn, -QuaternionOut) is semidet
%% tf_transform_pose(+SourceFrame, +TargetFrame, +PoseIn, -PoseOut) is semidet
% 
% Transforms input from SourceFrame to TargetFrame.
% PointIn is a list [float x, y, z].
% QuaternionIn is a list [float qx, qy, qz, qw].
% PoseIn is a term pose([float x, y, z], [float qx, qy, qz, qw]).
%%
:- use_foreign_library('librosprolog.so').
:- ros_init.

%% init_ros_package(+PackagePath) is nondet.
%
% Initialize a KnowRob package by consulting the prolog/init.pl file
% 
% @param PackagePath  Path towards the package to be initialized
% 
init_ros_package( PackagePath ) :-
  atom_concat(PackagePath, 'init.pl', InitFile),
  exists_file(InitFile),
  consult(InitFile), !.

init_ros_package( _ ).

%% register_ros_package(+Package) is nondet.
%% register_ros_package(+Package, ?AbsoluteDirectory) is nondet.
%
% Find and initialize a KnowRob package, i.e. locate the package on the
% harddisk, add the path to the library search path, and consult the init.pl
% file that (recursively) initializes the package and its dependencies.
% 
% @param Package            Name of a ROS package
% @param AbsoluteDirectory  Global path to the 'prolog' subdirectory in the given package
% 
register_ros_package(Package, _) :-
  current_predicate(ros_package_initialized/1),
  ros_package_initialized(Package), !.

register_ros_package(Package, AbsoluteDirectory) :-
  ros_package_path(Package, PackagePath),
  nonvar(PackagePath),
  atom_concat(PackagePath, '/prolog/', AbsoluteDirectory),
  asserta(library_directory(AbsoluteDirectory)),
  assert(user:file_search_path(ros, AbsoluteDirectory)),
  assert( ros_package_initialized(Package) ),
  init_classpath,
  init_ros_package( AbsoluteDirectory ).

register_ros_package(Package) :-
  register_ros_package(Package, _).

use_ros_module(Package, FilePath) :-
  register_ros_package(Package, AbsoluteDirectory),
  atom_concat(AbsoluteDirectory, FilePath, AbsoluteFilePath),
  use_module( AbsoluteFilePath ).

:- assert(classpath_initialized(false)).
init_classpath :-
  classpath_initialized(true), !.
init_classpath :-
  classpath_initialized(false),
  retract(classpath_initialized(false)),
  assert(classpath_initialized(true)),
  rosprolog_classpaths(Paths),
  setenv("CLASSPATH",Paths).

%% rosprolog_classpaths(+Paths) is nondet.
% 
% Read all classpath files from workspaces referenced in ROS_PACKAGE_PATH
%
% @param Paths The CLASSPATH environment variable
% 
rosprolog_classpaths(Paths) :-
  process_create(path('rosrun'), ['rosprolog', 'get_classpaths'], [stdout(pipe(Out)), process(PID)]),
  read_line_to_codes(Out, C),
  string_to_list(Paths, C),
  process_wait(PID, _).
