/*
  Copyright (C) 2016 Daniel Beßler
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
*/

:- module(knowrob_filesystem,
    [ path_delimiter/1,
      path_concat/3,
      path_split/2,
      mkdir/1
]).
/** <module> Filesystem utilities

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
% Makes sure that one filesytem delimiter is added time_between
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
