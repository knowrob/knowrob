%%
%% Copyright (C) 2011 by Martin Schuster
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

:- register_ros_package(ias_knowledge_base). %knowrob ontology
:- register_ros_package(ias_prolog_addons). %semantic distance
:- use_module(library('comp_similarity')).

% :- register_ros_package(comp_germandeli). %germandeli ontology
% :- register_ros_package(ias_semantic_map).
:- register_ros_package(comp_orgprinciples).
:- use_module(library('comp_orgprinciples')).

% :- use_module(library('comp_germandeli')).


% parse OWL files, register name spaces

% entities for testing, includes ias_semantic_map:ccrl2_semantic_map.owl to define the locations
% :- owl_parser:owl_parse('@LOCAL_PACKAGE_PATH@/owl/orgprinciples_demo.owl', false, false, true).
% :- rdf_db:rdf_register_ns(orgprinciples_demo, 'http://ias.cs.tum.edu/kb/orgprinciples_demo.owl#', [keep(true)]).

