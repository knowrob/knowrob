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

%%
url_resolve1(Path, Resolved) :-
    url_resolve(Path, Resolved),!.
url_resolve1(Path, Path).

%%
:- register_iri_scheme(file, file_iri_hook, []).

%!  file_iri_hook(+Action, +IRI, -Stream) is det.
%
%   Define the =|file://|= IRI scheme.
file_iri_hook(Operation, IRI, _) :-
    writeln(file_iri_hook(Operation, IRI)),
    fail.
file_iri_hook(open(Mode,Options), IRI, QueryPipelineStage) :-
    writeln(file_iri_hook(open(Mode,Options), IRI)),
    atom_concat('file://', Path, IRI),
    url_resolve1(Path, AbsPath),
    open(AbsPath, Mode, QueryPipelineStage, Options).
file_iri_hook(access(Mode), IRI, True) :-
    atom_concat('file://', Path, IRI),
    url_resolve1(Path, AbsPath),
    (   access_ok(Mode, AbsPath)
    ->  True=true
    ;   True=false
    ),
    writeln(file_iri_hook(access(Mode), AbsPath, True)).
file_iri_hook(time, IRI, Time) :-
    writeln(file_iri_hook(time, IRI)),
    atom_concat('file://', Path, IRI),
    url_resolve1(Path, AbsPath),
    time_file(AbsPath, Time),
    writeln(file_iri_hook(time, IRI, Time)).
file_iri_hook(size, IRI, Size) :-
    writeln(file_iri_hook(size, IRI)),
    atom_concat('file://', Path, IRI),
    url_resolve1(Path, AbsPath),
    size_file(AbsPath, Size),
    writeln(file_iri_hook(size, IRI, Size)).

%%
access_ok(directory, Path) :-
    !, exists_directory(Path).
access_ok(file, Path) :-
    !, exists_file(Path).
access_ok(_, _).
