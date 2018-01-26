%%
%% Copyright (C) 2018 by Moritz Tenorth
%%
%% This file contains tests for the URDF parsing tools in KnowRob.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- begin_tests(urdf_parsing).

:- use_module('urdf_parsing').

test(foo) :-
  findall(X, foo(X), Xs),
  Xs == [foo].

test(bar) :-
  findall(X, bar(X), Xs),
  Xs == [bar].

test(foo_bar) :-
  findall(X, foo_bar(X), Xs),
  Xs == [foo, bar].

:- end_tests(urdf_parsing).

