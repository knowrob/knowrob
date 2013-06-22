%%
%% Copyright (C) 2010 by Moritz Tenorth
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

% JPL
:- register_ros_package(jpl).

:- register_ros_package(ias_prolog_addons).

:- use_module(library(comp_similarity)).

% load and configure unit testing environment
:- use_module(library(plunit)).
:- set_test_options([load('always'),
                     run('make'),
                     silent(true)]).
