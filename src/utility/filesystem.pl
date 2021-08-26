:- module(utils_filesystem,
    [ path_delimiter/1,
      path_concat/3,
      path_split/2,
      mkdir/1,
      listdir/2
    ]).
/** <module> Interacting with the filesystem.

@author Daniel Beßler
@license BSD
*/

%% path_delimiter(?Delimiter)
%
% Delimiter for filesystem paths.
%
path_delimiter('/').

%% path_concat(+Prefix, +Suffix, ?Path)
%
% Concatenate path prefix with path suffix.
% Makes sure that one filesytem delimiter is added between
% Prefix and Suffix.
%
path_concat(Prefix, Suffix, Path) :-
  path_delimiter(Delimiter),
  (( atomic_list_concat(PrefixList, Delimiter, Prefix), last(PrefixList,'') )
  -> PrefixDelimited = Prefix
  ;  atomic_list_concat([Prefix,''], Delimiter, PrefixDelimited)
  ),
  (  atomic_list_concat([''|_], Delimiter, Suffix)
  -> atomic_list_concat(['',Suffix], Delimiter, SuffixRelative)
  ;  SuffixRelative = Suffix
  ),
  atom_concat(PrefixDelimited, SuffixRelative, Path).

%% path_split(?Path, ?PathList)
%
% Splits Path at delimiter characters and
% unifies with splitted path elements PathList.
%
path_split(Path, PathList) :-
  path_delimiter(Delimiter),
  atomic_list_concat(PathList, Delimiter, Path).
  
%% mkdir(+Dir)
%
% Create directory at Dir if it does not yet exist.
% Also creates all not-existing parent directories.
%
mkdir(Dir) :- exists_directory(Dir), !.
mkdir(Dir) :-
  path_split(Dir, [Head|Tail]),
  atom_concat('/', Head, Prefix),
  mkdir(Prefix, Tail),
  make_directory(Dir).

mkdir(Path, [Head|Tail]) :-
  (  exists_directory(Path)
  -> true
  ;  make_directory(Path)
  ),
  path_concat(Path, Head, ChildPath),
  mkdir(ChildPath, Tail).
mkdir(_, []).

%% listdir(+Dir, ?Entries)
%
% Get the contents of Dir without '.' and '..'
listdir(Dir, Entries) :-
  directory_files(Dir, Entries0),
  delete(Entries0, '.', Entries1),
  delete(Entries1, '..', Entries).
