
:- use_module(library('process')).

rospack_package_path(Package, Path) :-
  nonvar(Package),
  process_create(path('rospack'), ['find', Package], [stdout(pipe(RospackOutput)), process(_PID)]),
  read_line_to_codes(RospackOutput, C),
  string_to_list(Path, C).

init_ros_package( PackagePath ) :-
  atom_concat(PackagePath, 'init.pl', InitFile),
  exists_file(InitFile),
  consult(InitFile), !.

init_ros_package( _ ).

register_ros_package( Package, _ ) :-
  current_predicate(ros_package_initialized/1),
  ros_package_initialized(Package), !.

register_ros_package( Package, AbsoluteDirectory ) :-
  rospack_package_path(Package, PackagePath),
  nonvar(PackagePath),
  atom_concat(PackagePath, '/prolog/', AbsoluteDirectory),
  asserta(library_directory(AbsoluteDirectory)),
  assert(user:file_search_path(ros, AbsoluteDirectory)),
  assert( ros_package_initialized(Package) ),
  add_ros_package_to_classpath(Package),
  init_ros_package( AbsoluteDirectory ).


register_ros_package( Package ) :-
  register_ros_package( Package, _ ).

use_ros_module( Package, FilePath ) :-
  register_ros_package(Package, AbsoluteDirectory),
  atom_concat(AbsoluteDirectory, FilePath, AbsoluteFilePath),
  use_module( AbsoluteFilePath ).

% adds java dependencies to classpath
add_ros_package_to_classpath(Package):-
	rospack_package_classpath(Package, Path),
	atom_concat(':',Path,PackagePath),
	concat_env("CLASSPATH",PackagePath).

% calculates java dependencies for classpath
rospack_package_classpath(Package, Path) :-
  nonvar(Package),
  process_create(path('rospack'), ['export', '--lang=java', '--attrib=classpath', Package], [stdout(pipe(RospackOutput)), process(_PID)]),
  read_line_to_codes(RospackOutput, C),
  string_to_list(String, C),
  concat_atom(List,' ',String),% split string at ' '
  concat_atom(List,':',Path).  % concat list elements with seperator ':'

% concat a value to an environment varible
% please note: delimiters have to be set within Val, e.g.:
% ':/path/to/lib:/path/to/lib2'
concat_env(Var,Val):-
	(getenv(Var,OldVal)
	->  (atom_concat(OldVal,Val,NewVal)) ;
        (NewVal = Val)),
	setenv(Var,NewVal).
