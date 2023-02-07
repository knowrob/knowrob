:- module(url,
        [ url_resolve/2
        ]).
/** <module> Resolving URLs to local filesystem.

@author Daniel Beßler
@license BSD
*/

:- multifile url_resolve/2.
:- dynamic project_directory/1.

:- use_module('filesystem', [ path_concat/3 ]).

:- prolog_load_context(directory, PrologDir),
   once(( atom_concat(ProjectDir, '/src/reasoner/prolog', PrologDir)  % loaded from source tree
        ; atom_concat(ProjectDir, '/reasoner/prolog', PrologDir)      % loaded installed file
   )),
   assert(project_directory(ProjectDir)).

%% url_resolve(+URL, -Resolved) is nondet.
%
% A multifile predicate used to support special URL schemas
% like ROS package URLs of the form "package://rospkg/....".
%
url_resolve(ProjectPath, Resolved) :-
    % Resolve project-relative paths, i.e. files that are located relative to this Prolog file.
    \+ atom_prefix(ProjectPath, '/'),
    \+ atom_prefix(ProjectPath, 'http:'),
    ( atom_concat('./', ProjectPath0, ProjectPath) -> true
    ; ProjectPath0 = ProjectPath ),
    project_directory(ProjectRoot),
    path_concat(ProjectRoot, ProjectPath0, Resolved),
    exists_file(Resolved).
