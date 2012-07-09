/** <module> knowrob_cad_parser

  Description:
    Module providing mesh predicates for parsing cad models

  Copyright (C) 2012 by Stefan Profanter

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Stefan Profanter
@license GPL
*/

:- module(knowrob_cad_parser,
    [
		get_model_path/2
    ]).	

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Main
%


%% get_model_path(+Identifier,-Path) is det.
%
% searches for knowrob:pathToCadModel property
%
% @param Identifer	Object identifier
% @param Path		Found path
get_model_path(Identifier,Path) :-
	(rdf_has(Identifier,knowrob:pathToCadModel,literal(type(_,Path))) ; 
	owl_individual_of(Identifier, Class), class_properties(Class, knowrob:pathToCadModel,literal(type(_,Path)));
	rdf_has(Identifier,knowrob:pathToCadModel,literal(Path)) ; 
	owl_individual_of(Identifier, Class), class_properties(Class, knowrob:pathToCadModel,literal(Path))),!.
	
