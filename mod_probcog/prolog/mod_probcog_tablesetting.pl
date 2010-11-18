/** <module> mod_probcog_tablesetting

  Description and wrapper functions for the ProbCog tablesetting model

  This module is a sub-module of mod_probcog and defines model-specific
  predicates for
  - reading evidence
  - combining single queries to perform advanced inference
  - determine if predicates are to be regarded as open or closed world


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

:- module(mod_probcog_tablesetting,
    [
      required_objects/1
    ]).

:- use_module(library('jpl')).
:- use_module(library('mod_probcog')).
:- use_module(library('comp_temporal')).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Wrapper predicates for reading evidence
%
% Note that these predicates are NOT exported in order to avoid conflicts between different
% implementations for different models. For calling them, use mod_probcog_bla:predname
%

% 
% person P uses utensil U during meal M
% 
% usesAnyIn(P, U, M) :-
%   rdfs_individual_of(P, knowrob:'Person'),
%   (
%     rdfs_individual_of(U, knowrob:'FoodVessel');
%     rdfs_individual_of(U, knowrob:'PhysicalDevice')
%   ),
%   rdfs_individual_of(M, knowrob:'Meal'),
% 
%   % find objects that are the objectActedOn of an
%   % ActionOnObject during the meal M
%   rdfs_individual_of(A, knowrob:'ActionOnObject'),
%   rdf_triple(knowrob:performedBy, A, P),
%   rdf_triple(knowrob:temporallySubsumes, M, A),
%   rdf_triple(knowrob:objectActedOn, A, U).

% shortcut: use everything on a table
usesAnyIn(person1, U, meal1) :-

  table_to_complete(Table),
  comp_missingobj:current_objects_on_table(Table, U),

  ( rdfs_individual_of(U, knowrob:'FoodVessel');
    rdfs_individual_of(U, knowrob:'PhysicalDevice') ).


%
% person P consumes food F during meal M
%
consumesAnyIn(person1, F, meal1) :-
  table_to_complete(Table),
  comp_missingobj:current_objects_on_table(Table, F),
  rdfs_individual_of(F, knowrob:'FoodOrDrink').


%
% person P uses utensil U during meal M with consumable F
%
usesAnyForIn(P, U, F, M) :-
  rdfs_individual_of(P, knowrob:'Person'),
  (
    rdfs_individual_of(U, knowrob:'FoodVessel');
    rdfs_individual_of(U, knowrob:'PhysicalDevice')
  ),
  rdfs_individual_of(M, knowrob:'Meal'),
  rdfs_individual_of(F, knowrob:'FoodOrDrink'),

  % find objects that are the objectActedOn of an
  % ActionOnObject during the meal M
  rdfs_individual_of(A, knowrob:'ActionOnObject'),
  rdf_triple(knowrob:performedBy, A, P),
  rdf_triple(knowrob:temporallySubsumes, M, A),
  rdf_triple(knowrob:objectActedOn, A, U).


%
% person P sits at location L during meal M
%
sitsAtIn(P, _, M) :-
  rdfs_individual_of(P, knowrob:'Person'),
  rdfs_individual_of(M, knowrob:'Meal'),
  fail. % TODO

%
% person P takes part in meal M
%
% Dummy: person1 always takes part in meal1
%
takesPartIn(person1, meal1).

g(_) :- fail.
u(_) :- fail.
pl(_) :- fail.

%
% utensil U commonly used by participants of meal M
%
anyUsedByAllIn(_, _) :-
  fail.

%
% person P has name N
%
:- redefine_system_predicate(name(_,_)).
name(P, N) :-
  rdfs_individual_of(P, knowrob:'Person'),
  rdf_split_url(_, N, P),
  fail. % TODO

%
% meal M has type T
%
% dummy: meal1 is breakfast
% 
mealT(meal1,'Breakfast').
mealT(M, T) :-
  rdfs_individual_of(M, knowrob:'Meal'),
  rdf_has(M, rdf:type, T),
  fail.

%
% meal M takes place on day D
%
day(M, _) :-
  rdfs_individual_of(M, knowrob:'Meal'),
  fail. % TODO

%
% meal M takes place at time T
% 
timeT(M, T) :-
  rdfs_individual_of(M, knowrob:'Meal'),
  rdf_has(M, knowrob:startTime, T),
  fail.




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Wrapper predicates for advanced queries
%


%% required_objects(-Obj) is nondet.
%
% Reads the objects that are required at the table
%
% @param Obj  Objects required at this place
%
:- assert(table_to_complete('http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable0')).

required_objects(Instances) :-
  required_objects('http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable0', Instances).

required_objects(Table, Instances) :-

  nonvar(Table),
  retractall(table_to_complete(_)),
  assert(table_to_complete(Table)),!,

  probcog_query(['usesAnyIn(person1, ?, meal1)', 'consumesAnyIn(person1, ?, meal1)'], [First|[Second|_]]), append(First,Second,Types),

  predsort(compare_inference_probs, Types, Types_sorted),
  get_time(Time),
  create_instances(Types_sorted,Time, Instances),!.


% create only instances above a certain probability threshold
create_instances([T|Types], Time, [I|Instances]) :-
  T= [_|Prob], [P]=Prob,
  term_to_atom(ProbNum, P), =<(0.6, ProbNum),
  create_instance(T, Time, I),
  create_instances(Types, Time, Instances).

create_instances([T|Types], Time, Instances) :-
  T= [_|Prob], [P]=Prob,
  term_to_atom(ProbNum, P), >(0.6, ProbNum),
  create_instances(Types, Time, Instances).

create_instances([], _, []).


create_instance([Type|Prob], Time, Inst) :-

  % create inference object
  rdf_instance_from_class('http://ias.cs.tum.edu/kb/knowrob.owl#TableSettingModelInference', Perception),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_', Time, TimePoint),
  rdf_assert(TimePoint, rdf:type, knowrob:'TimePoint'),
  rdf_assert(Perception, knowrob:startTime, TimePoint),

  % create object instance
  string_to_atom(Type, TypeAtom),
  rdf_instance_from_class(TypeAtom, Inst),
  rdf_assert(Perception, knowrob:objectActedOn, Inst),

  % set the probability
  [P]=Prob,
  rdf_assert(Perception, knowrob:probability, P),

  % set the pose
  next_default_pose(Pose),
  rdf_assert(Inst, knowrob:orientation, Pose).


%% compare_inference_probs(-Delta, +P1, +P2)
%
% compare two object classes with their resp. probabilities
%
compare_inference_probs('>', [_, P1], [_, P2]) :-
    term_to_atom(N1, P1),
    term_to_atom(N2, P2),
    N1 < N2.

compare_inference_probs('<', [_, P1], [_, P2]) :-
    term_to_atom(N1, P1),
    term_to_atom(N2, P2),
    N1>=N2.


%% next_default_pose(-Pose) is det.
%
% read one of a set of predefined poses where the (inferred) objects are to be put on the table
%

:- assert(default_pose_nr(0)).
next_default_pose(Pose):-

  % retrieve global index
  default_pose_nr(Index),

  Poses = ['http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.75_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.55_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.35_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.15_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-0.95_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.9_0_1_0_-1.75_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.9_0_1_0_-1.55_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.9_0_1_0_-1.35_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.9_0_1_0_-1.15_0_0_1_1.00_0_0_0_1',
           'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.9_0_1_0_-0.95_0_0_1_1.00_0_0_0_1'],

%   Poses = ['http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.5_0_1_0_-0.95_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.5_0_1_0_-1.15_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.5_0_1_0_-1.35_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.5_0_1_0_-1.55_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.5_0_1_0_-1.75_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-0.95_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.15_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.35_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.55_0_0_1_0.90_0_0_0_1',
%            'http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D_1_0_0_2.7_0_1_0_-1.75_0_0_1_0.90_0_0_0_1'],


  nth0(Index, Poses, Pose),

  rdf_assert(Pose, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#RotationMatrix3D'),

  % update index
  retract(default_pose_nr(_)),
  length(Poses, NumPoses),
  Index1 is ((Index+1) mod NumPoses),
  assert(default_pose_nr(Index1)),!.



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Definition of open/closed world predicates
%
% Default: regard everything as open-world if there are no predicates defined here
% 
% The computation if one predicate is to be regarded as closed world may be rather complex,
% taking the query context into account.
%
% general form: 
% closed_world(Pred, QueryContext) :- ...
%
% query-independently closed world:
% closed_world('predname', _).


