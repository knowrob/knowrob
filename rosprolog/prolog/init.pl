
:- use_module(library('process')).

%% rospack_package_path(+Package, -Path) is nondet.
%
% Locate ROS packages on the harddisk using 'rospack find'
% 
% @param Package  Name of a ROS package
% @param Path     Global path of Package determined by 'rospack find'
% 
rospack_package_path(Package, Path) :-
  nonvar(Package),
  process_create(path('rospack'), ['find', Package], [stdout(pipe(RospackOutput)), process(PID)]),
  read_line_to_codes(RospackOutput, C),
  string_to_list(Path, C),
  process_wait(PID, _).

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
  rospack_package_path(Package, PackagePath),
  nonvar(PackagePath),
  atom_concat(PackagePath, '/prolog/', AbsoluteDirectory),
  asserta(library_directory(AbsoluteDirectory)),
  assert(user:file_search_path(ros, AbsoluteDirectory)),
  assert( ros_package_initialized(Package) ),
  add_ros_package_to_classpath(Package),
  init_ros_package( AbsoluteDirectory ).


register_ros_package(Package) :-
  register_ros_package(Package, _).

use_ros_module(Package, FilePath) :-
  register_ros_package(Package, AbsoluteDirectory),
  atom_concat(AbsoluteDirectory, FilePath, AbsoluteFilePath),
  use_module( AbsoluteFilePath ).



%% add_ros_package_to_classpath(+Package) is nondet.
% 
% Adds Java dependencies of Package to the CLASSPATH environment variable
%
% @param Package Name of a ROS package
% 
add_ros_package_to_classpath(Package):-
	rospack_package_classpath(Package, Path),
	atom_concat(':',Path,PackagePath),
	setenv("CLASSPATH",PackagePath).

%% rospack_package_classpath(+Package, -Path) is nondet.
% 
% Calculates the Java dependencies of Package and returns a string to be appended to the CLASSPATH
%
% @param Package  Name of a ROS package
% @param Path     String with the dependencies to be added to the CLASSPATH
% 
rospack_package_classpath(Package, Path) :-
  nonvar(Package),
  process_create(path('rosrun'), ['rosprolog', 'get_pkg_classpath', Package], [stdout(pipe(RospackOutput)), process(PID)]),
  read_line_to_codes(RospackOutput, C),
  string_to_list(Path, C),
  process_wait(PID, _).

% concat a value to an environment varible
% please note: delimiters have to be set within Val, e.g.:
% ':/path/to/lib:/path/to/lib2'
concat_env(Var,Val):-
	(getenv(Var,OldVal)
	->  (atom_concat(OldVal,Val,NewVal)) ;
        (NewVal = Val)),
	setenv(Var,NewVal).
