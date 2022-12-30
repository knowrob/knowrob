:- module(utils_url,
        [ ros_package_iri/2
        ]).
/** <module> Resolving URLs to local filesystem.

@author Daniel Beßler
@license BSD
*/

:- use_module('filesystem.pl', [ path_concat/3 ]).

:- dynamic ros_package_iri_/2.

%%
url_resolve(A,B) :- ros_resolve(A,B),!.

%%
ros_resolve(PackagePath,LocalPath) :-
  ros_path(PackagePath,LocalPath).

ros_resolve(URL,LocalPath) :-
  %% map IRI to ROS package path based on predicate *ros_package_iri_*
  file_base_name(URL,FileName),
  file_directory_name(URL,Prefix),
  ros_package_iri_(Pkg,Prefix),
  ros_package_path(Pkg,PkgPath),
  %% convention is that rdf files are stored in a directory named "owl" or "rdf"
  ( path_concat(PkgPath,owl,Dir);
    path_concat(PkgPath,'owl/external',Dir);
    path_concat(PkgPath,rdf,Dir);
    Dir=PkgPath
  ),
  path_concat(Dir,FileName,LocalPath),
  exists_file(LocalPath),
  !.

%% ros_package_iri(+PkgName,+URI) is det.
%
% Register an IRI for a ROS package.
% When RDF files are loaded with the IRI prefix,
% it is first tried to serve the local file from
% the ROS package before downoading the file
% from the web.
%
ros_package_iri(PkgName,URI) :-
  assertz(ros_package_iri_(PkgName,URI)).
