/** <module> mod_probcog

  Interface to the ProbCog probabilistic reasoning framework

  Copyright (C) 2010 by Moritz Tenorth

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

@author Moritz Tenorth
@license GPL
*/

:- module(mod_probcog,
    [
      probcog_query/2,
      probcog_query/3
    ]).

:- use_module(library('jpl')).



%% probcog_query(+Model, +QueryPredicates, -Res) is nondet.
% 
% Send a query to the ProbCog probabilistic reasoning engine using the
% specified model for the given set of query predicates.
%
% The result is a list of answers for each query, which by themselves are
% lists of bindings. Example:
% [ [['q1_value1', 'q1_prob1'], ['q1_value2', 'q1_prob2']], [['q2_value1', 'q2_prob1']] ]
% 
% There are two kinds of queries: When asking for a predicate with query variables
% (marked with a preceding '?'), such as
% 
% ?- probcog_query(['usesAnyIn(person1, ?, meal1)', 'sitsAtIn(person1, ?, meal1)'], Res).
% 
% the system returns only the bindings for each of these variables, together with
% their probabilities. If there are several query variables, the bindings are joined
% with an underscore. Example:
%
% ?- probcog_query(['usesAnyIn(person1, ?, meal1)', 'sitsAtIn(person1, ?, meal1)'], Res).
% Res = [[['TableKnife_meal1', '1.0'], ['Bowl-Eating_meal1', '0.638'],], [['Seat3', '0.17'], ['Seat1', '0.41']]]
%
% The second kind of query asks for all possible bindings of a predicate and returns
% the complete term:
% 
% ?- probcog_query(['usesAnyIn'], Res).
% Res = [[['usesAnyIn(person1, TableKnife, meal1)', '1.0'], ['usesAnyIn(person1, Bowl-Eating, meal1)', '0.67']]]
% 
%
% Evidence is read by the performInference procedure depending on the model to be
% used and the wrapper predicates specified in the corresponding Prolog module.
%
% @param Model           The model that is to be used for inference
% @param QueryPredicates Predicates whose most likely values are to be inferred, List of the form [q1, q2, q3]
% @param Res             Variable bindings for the query predicates as a list [[binding_1_q1,binding_1_q2,binding_1_q3],[binding_2_q1,binding_2_q2,binding_2_q3],..]
% 
probcog_query(Model, QueryPredicates, Res) :-
  jpl_list_to_array(QueryPredicates, Array),
  probcog_module_name(Model, ModuleName),
  jpl_call('edu.tum.cs.probcog.prolog.PrologInterface', 'performInference', [Model, ModuleName, Array], ResArray),
  arrays_to_lists(ResArray, Res).


%% probcog_query(+QueryPredicates, -Res) is nondet.
%
% Send a query to the ProbCog probabilistic reasoning engine using the default
% model for the given set of query predicates.
%
% @param QueryPredicates Predicates whose most likely values are to be inferred, List of the form [q1, q2, q3]
% @param Res             Variable bindings for the query predicates as a list [[binding_1_q1,binding_1_q2,binding_1_q3],[binding_2_q1,binding_2_q2,binding_2_q3],..]
% 
probcog_query(QueryPredicates, Res) :-
  probcog_model(QueryPredicates, Model),
  probcog_query(Model, QueryPredicates, Res).


%% probcog_model(+QueryPredicates, -Model) is det.
%
% Determine the default model for a set of query predicates.
%
probcog_model(_, 'tableSetting_fall10').


%% Mapping from model name to corresponding prolog module name
%
% Required for JPL interface to query evidences properly
probcog_module_name('tableSetting_fall10', 'mod_probcog_tablesetting').
