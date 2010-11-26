
:- style_check(-discontiguous).

:- ['~/.plrc'].

:- use_module(library('process')).


:- dynamic   user:file_search_path/2.
:- multifile user:file_search_path/2.


doc_rospack_package_path(Package, Path) :-
  nonvar(Package),
  process_create(path('rospack'), ['find', Package], [stdout(pipe(RospackOutput)), process(_PID)]),
  read_line_to_codes(RospackOutput, C),
  string_to_list(Path, C).

doc_register_ros_package( Package, AbsoluteDirectory ) :-
  doc_rospack_package_path(Package, PackagePath),
  nonvar(PackagePath),
  atom_concat(PackagePath, '/prolog/', AbsoluteDirectory),
  assertz(library_directory(AbsoluteDirectory)),
  assert(user:file_search_path(ros, AbsoluteDirectory)).


% add rosprolog/prolog directory to the library search path
:- doc_register_ros_package(rosprolog,_).

:- load_files([ library(pldoc),
		library(doc_http),
		library(doc_latex),
		library(http/http_error),
		library('pldoc/doc_files')
	      ],
	      [ silent(true),
		if(not_loaded)
	      ]).
 :- debug(pldoc).


% :- use_module(library('pldoc')).
% :- use_module(library('pldoc/doc_files')).


