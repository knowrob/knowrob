/** <module> tabletop_obj

  This module provides routines to interface the the tabletop_object_detector
  system, i.e. to read data and interpret the results.

  Copyright (C) 2010 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a ttoy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(comp_battery_state,
    [
      cbattery_state/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(battery_state, 'http://ias.cs.tum.edu/kb/battery_state.owl#', [keep(true)]).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% As part of the KnowRob tutorial, you are supposed to implement the
% following two predicates. You can use the predicates below for
% creating the object and perception instances.
%

cbattery_state(Robot, Power) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_battery_state.ROSClient', ['my_battery_client'], Client),

    % call the method for retrieving objects from the tabletop_object_detector
    jpl_call(Client, 'getPowerByPooling', [], Returned_Power),

    term_to_atom(Returned_Power, Y),
    rdf_assert(Y, rdf:type, xsd:'float'),

    % rdf_has(Robot, rdf:type, knowrob:'PR2'),
    rdf_has(Power, rdf:type, xsd:'float'),
 
    atom_to_term(Power, R, []),
    
    A is round(R),
    B is round(Returned_Power),
    
    A = B.


    % rdf_has(Robot, rdf:type, knowrob:'PR2'),
    % X is 0,
    % term_to_atom(X, Y),

    % rdf_assert(Y, rdf:type, xsd:'float'),

    % rdf_has(Power, rdf:type, xsd:'float'),

    % create ROS client object
    % jpl_new('edu.tum.cs.ias.knowrob.mod_battery_state.ROSClient', ['my_battery_client'], Client),

    % call the method for retrieving objects from the tabletop_object_detector
    % jpl_call(Client, 'getPowerByPooling', [], Returned_Power),

    % convert the result into a list of matches over which we can iterate
    % jpl_get(Objects, 'models', Models),

    % term_to_atom(P, Returned_Power),
     
    % atom_to_term(Power, R, []),
    
    % A is round(R),
    
    % B is round(Returned_Power),
    
    % A = B.


