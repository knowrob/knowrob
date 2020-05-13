:- module(utils_url,
        [ url_resolve/2,
          url_unresolve/2,
          ros_package_iri/2
        ]).
/** <module> TODO

@author Daniel Beßler
@license BSD
*/

:- dynamic resolve/2.
:- dynamic ros_package_iri_/2.

%%
%
%
url_resolve(URL,Resolved) :-
  resolve(URL,Resolved),!.

%%
%
%
url_unresolve(URL,Unresolved) :-
  resolve(Unresolved,URL),!.

%%
%
% TODO: move into ROS-specific comm package
%
resolve(A,B) :-
  var(A) -> ros_unresolve(B,A) ; ros_resolve(A,B).

%%
ros_unresolve(PackagePath,URL) :-
  atom_concat('package://',Path,PackagePath),
  file_base_name(PackagePath,FileName),
  atomic_list_concat([Pkg|_],'/',Path),
  ros_package_iri_(Pkg,Prefix),!,
  atomic_list_concat([Prefix,FileName],'/',URL).

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
    path_concat(PkgPath,rdf,Dir);
    path_concat(PkgPath,Dir)
  ),
  path_concat(Dir,FileName,LocalPath),
  exists_file(LocalPath).

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
