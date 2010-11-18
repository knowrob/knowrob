/** <module> comp_objclusters

  Description:
    classify point cloud clusters in order to find sensible hypotheses of the
    object class they belong to

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


:- module(comp_objclusters,
    [
      comp_objclass/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('classifiers')).

:- use_module(library('comp_ros')).

:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).


%% comp_objclass(+ObjClass, +InputLoIDs, -SortedLoIDs) is det.
%
% Procedure:
% * predicate gets the desired object class and a list of candidate loIDs
% * uses classifier to find the probability distribution over object classes for each loID
% * sort the list of candidate IDs based on the prob that they are of the desired class
%
% @param ObjClass Identifier of the desired class of object
% @param InputLoIDs List of candidate IDs
% @param SortedLoIDs List of lo-IDs, sorted by the likelihood that they correspond to the desired object class

comp_objclass(ObjClass, InputLoIDs, SortedLoIDs) :-

  % load saved classifier
  classifier_saved('rec2adaJ48.clfwrp', Classifier),

  % iterate across loIDs
  findall(F, (member(LoID, InputLoIDs), obj_features(LoID, F)), Features),
  
  % get distribution over class hypotheses
  distribution_for_instances(Classifier, Features, Distribution),

  % add loIDs to each distribution
  typed_distrib(Distribution, InputLoIDs, TypedDist),

  % find the probability that belongs to ObjClass
  filter_distrib(TypedDist, ObjClass, Filtered),

  % sort the list of loIDs according to the prob
  predsort(hypoDistCompare, Filtered, SortedDist),
  get_lo_ids(SortedDist, SortedLoIDs).



%
% create a list of only the lo IDs
%
get_lo_ids([D|Dist], [I|IDs]) :-
    nth0(0, D, I),
    get_lo_ids(Dist, IDs).
get_lo_ids([], []).





%
% select the element in the hypo list that belongs to the desired ObjClass
%
filter_distrib([D|Dist], ObjClass, [F|Filtered]) :-

    nth0(0,   D, LoID),
    nth0(Ids, D, [ObjClass,_]),
    nth0(Ids, D, Res),
    F=[LoID, Res],
    filter_distrib(Dist, ObjClass, Filtered).
filter_distrib([], _, []).




hypoDistCompare(Delta, [_,[_, P1]], [_,[_, P2]]) :- 
    term_to_atom(N1, P1),
    term_to_atom(N2, P2),
    compare(Delta, N2, N1).


%
% calculate the features used by the classifier
%
obj_features(LoID, Features) :-

  query_lo_orientation('framequery', '', LoID, 14, Pose),
  query_lo_covariance('framequery',  '', LoID, 14, Cov),
  
  rdf_triple(knowrob:m00, Pose, P00),strip_literal_type(P00, Pose00),
  rdf_triple(knowrob:m01, Pose, P01),strip_literal_type(P01, Pose01),
  rdf_triple(knowrob:m02, Pose, P02),strip_literal_type(P02, Pose02),
  rdf_triple(knowrob:m03, Pose, P03),strip_literal_type(P03, Pose03),
  rdf_triple(knowrob:m10, Pose, P10),strip_literal_type(P10, Pose10),
  rdf_triple(knowrob:m11, Pose, P11),strip_literal_type(P11, Pose11),
  rdf_triple(knowrob:m12, Pose, P12),strip_literal_type(P12, Pose12),
  rdf_triple(knowrob:m13, Pose, P13),strip_literal_type(P13, Pose13),
  rdf_triple(knowrob:m20, Pose, P20),strip_literal_type(P20, Pose20),
  rdf_triple(knowrob:m21, Pose, P21),strip_literal_type(P21, Pose21),
  rdf_triple(knowrob:m22, Pose, P22),strip_literal_type(P22, Pose22),
  rdf_triple(knowrob:m23, Pose, P23),strip_literal_type(P23, Pose23),
  
  rdf_triple(knowrob:m00, Cov, C00), strip_literal_type(C00, Cov00),
  rdf_triple(knowrob:m11, Cov, C11), strip_literal_type(C11, Cov11),
  rdf_triple(knowrob:m22, Cov, C22), strip_literal_type(C22, Cov22),

  Features = [Pose00, Pose01, Pose02, Pose03, Pose10, Pose11, Pose12, Pose13, Pose20, Pose21, Pose22, Pose23, Cov00, Cov11, Cov22].

%
% util: remove the literal type and extract the content
%
strip_literal_type(literal(type(_, Value)), Value) :- !.
strip_literal_type(Value, Value).


% 
% add loID as first element of each list
% 
% [
%   [LoID1, [cup, '0.079'],   [thermos, '1.43E-5'], [icetea, '0.920'], [teabox, '1.50E-4']],
%   [LoID2, [cup, '8.97E-6'], [thermos, '1.67E-5'], [icetea, '0.999'], [teabox, '1.29E-4']]
% ]
typed_distrib([D|Dist], [L|LoIDS], [T|TypedDist]) :- 
  T=[L|D],
  typed_distrib(Dist, LoIDS, TypedDist).
typed_distrib([], [], []).


