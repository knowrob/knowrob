/*
  Copyright (C) 2015 Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
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

:- module(data_vis,
    [
      data_vis/2,
      data_vis_remove/1,
      timeline/1
    ]).
/** <module> Methods for data visualization
  
  The visualization artifacts are supposed to be generated client side.
  KnowRob only generates a data structure holding the data and
  data visualization clients can connect to the topic on which
  KnowRob publishes the visualization data.

  @author Moritz Tenorth
  @author Daniel Beßler
  @license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('jpl')).

%% data_vis(+Term:term, +Properties:list) is det
%
% Creates a new data_vis message and publishes it via _|/data_vis_msgs|_
% ROS topic.
% Term is one of piechart(Identifier), barchart(Identifier),
% treechart(Identifier), timeline(Identifier), or linechart(Identifier).
% Properties is a list of properties for the data_vis message of the form key:value.
% Allowed keys are: data, title, xlabel, ylabel, width, height, fontsize.
%
% @param Term the data_vis term
% @param Properties list of data_vis properties
%
data_vis(Term, Properties) :-
  data_vis_object(Term, Object),
  data_vis_set_properties(Object, Properties),
  data_vis_publish.

%% timeline(+Events:list) is det
%
% Creates a new data_vis timeline message and publishes it via _|/data_vis_msgs|_
% ROS topic.
%
% @param Events list of temporal extended things
%
timeline(Events) :-
  findall(Time, (
     member(Evt, Events),
     interval(Evt, Interval),
     atomic_list_concat(Interval, '_', Time)
  ), EventExtends),
  data_vis(timeline(event_timeline),
           data([[Events,EventExtends]])).


%% data_vis_remove(+Identifier) is det.
%
% Republishes a data_vis object with empty data
% so that visualization clients can remove it.
%
data_vis_remove(all) :- !,
  data_vis_remove_objects.
data_vis_remove(Identifier) :-
  data_vis_remove_object(Identifier).

%% data_vis_object
data_vis_object(piechart(Identifier), Object) :-
  data_vis_new_object(Identifier, Object),
  data_vis_set_type(Object, 0).
data_vis_object(barchart(Identifier), Object) :-
  data_vis_new_object(Identifier, Object),
  data_vis_set_type(Object, 1).
data_vis_object(treechart(Identifier), Object) :-
  data_vis_new_object(Identifier, Object),
  data_vis_set_type(Object, 2).
data_vis_object(timeline(Identifier), Object) :-
  data_vis_new_object(Identifier, Object),
  data_vis_set_type(Object, 3),
  data_vis_set_property(Object, xlabel:'Time'),
  data_vis_set_property(Object, ylabel:'Events'),
  data_vis_set_property(Object, fontsize:'12px').
data_vis_object(linechart(Identifier), Object) :-
  data_vis_new_object(Identifier, Object),
  data_vis_set_type(Object, 4).

%% data_vis_set_type
data_vis_set_type(DataVisObject, Type) :-
  jpl_call(DataVisObject, setType, [Type], _).

%% data_vis_set_data
data_vis_set_data(DataVisObject, Data) :-
  data_vis_set_property(DataVisObject, data:Data).

%% data_vis_set_property
data_vis_set_property(DataVisObject, data:[Data,Labels]) :-
  lists_to_arrays(Data, DataArr),
  lists_to_arrays(Labels, LabelsArr),
  jpl_call(DataVisObject, setData, [DataArr,LabelsArr], _), !.
data_vis_set_property(DataVisObject, data:Data) :-
  lists_to_arrays(Data, DataArr),
  jpl_call(DataVisObject, setData, [DataArr], _), !.
data_vis_set_property(DataVisObject, title:Title) :-
  jpl_call(DataVisObject, setTitle, [Title], _), !.
data_vis_set_property(DataVisObject, xlabel:Label) :-
  jpl_call(DataVisObject, setXLabel, [Label], _), !.
data_vis_set_property(DataVisObject, ylabel:Label) :-
  jpl_call(DataVisObject, setYLabel, [Label], _), !.
data_vis_set_property(DataVisObject, width:Label) :-
  jpl_call(DataVisObject, setWidth, [Label], _), !.
data_vis_set_property(DataVisObject, height:Label) :-
  jpl_call(DataVisObject, setHeight, [Label], _), !.
data_vis_set_property(DataVisObject, fontsize:Label) :-
  jpl_call(DataVisObject, setFontsize, [Label], _), !.
data_vis_set_property(_, _).

%% data_vis_set_properties
data_vis_set_properties(_, []) :- !.
data_vis_set_properties(DataVisObject, [Prop|Rest]) :-
  data_vis_set_property(DataVisObject, Prop),
  data_vis_set_properties(DataVisObject, Rest).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% ROS node for marker visualization
%

data_visualisation :-
  data_visualisation(_).
  
data_visualisation(DataVis) :-
  (\+ current_predicate(v_data_vis, _)),
  jpl_call('org.knowrob.vis.DataVisPublisher', get, [], DataVis),
  jpl_list_to_array(['org.knowrob.vis.DataVisPublisher'], Arr),
  jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [DataVis, Arr], _),
  assert(v_data_vis(DataVis)),!.

data_visualisation(DataVis) :-
  current_predicate(v_data_vis, _),
  v_data_vis(DataVis).

data_vis_publish :-
  data_visualisation(DataVis),
  jpl_call(DataVis, publish, [], _).

data_vis_new_object(Identifier, DataVisObject):-
  data_visualisation(DataVis),
  jpl_call(DataVis, createDataVisObject, [Identifier], DataVisObject).

data_vis_remove_object(Identifier):-
  data_visualisation(DataVis),
  jpl_call(DataVis, removeDataVisObject, [Identifier], _).

data_vis_remove_objects :-
  data_visualisation(DataVis),
  jpl_call(DataVis, removeDataVisObjects, [], _).

