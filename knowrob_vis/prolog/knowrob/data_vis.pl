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
  data_vis_object(Term, Object), !,
  data_vis_set_properties(Object, Properties),
  data_vis_publish(Object).

%% timeline(+Events:list) is det
%
% Creates a new data_vis timeline message and publishes it via _|/data_vis_msgs|_
% ROS topic.
%
% @param Events list of temporal extended things
%
timeline(Events) :-
  findall(EvtName, (
    member(Evt,Events),
    once((
      rdfs_type_of(Evt,EvtType),
      rdf_split_url(_,EvtName,EvtType))),
    interval(Evt, [_,_])
  ), EvtNames),
  findall(Time, (
    member(Evt, Events),
    once((
      interval(Evt, [T0,T1]),
      atomic_list_concat([T0,T1], '_', Time)))
  ), EventExtends),
  data_vis(timeline(event_timeline),
          [data:[[EvtNames,EventExtends]]]).

%% data_vis_object
data_vis_object(piechart(Identifier), Object) :-
  data_vis_(Identifier,0,Object).
data_vis_object(barchart(Identifier), Object) :-
  data_vis_(Identifier,1,Object).
data_vis_object(treechart(Identifier), Object) :-
  data_vis_(Identifier,2,Object).
data_vis_object(timeline(Identifier), Object) :-
  data_vis_(Identifier,3,Object),
  b_set_dict(title, Object, 'Logged Actions'),
  b_set_dict(xlabel, Object, 'Time'),
  b_set_dict(ylabel, Object, 'Events').
data_vis_object(linechart(Identifier), Object) :-
  data_vis_(Identifier,4,Object).

data_vis_(Identitier, Type, _{
  id: Identitier,
  type: Type,
  title: '',
  xlabel: '',
  ylabel: '',
  width: 200,
  height: 200,
  fontsize: '12px',
  values: []
}).

%% data_vis_set_data
data_vis_set_data(Object, Data) :-
  data_vis_set_property(Object, data:Data).

%% data_vis_set_property
data_vis_set_property(Object, data:[Data,Labels]) :-
  lists_to_arrays(Data, DataArr),
  lists_to_arrays(Labels, LabelsArr),!,
  b_set_dict(data, Object, [DataArr,LabelsArr]).
data_vis_set_property(Object, data:Data) :-
  lists_to_arrays(Data, DataArr),!,
  b_set_dict(data, Object, DataArr).
data_vis_set_property(Object, Key:Value) :-
  b_set_dict(Key, Object, Value).

%% data_vis_set_properties
data_vis_set_properties(_, []) :- !.
data_vis_set_properties(Object, [Prop|Rest]) :-
  data_vis_set_property(Object, Prop), !,
  data_vis_set_properties(Object, Rest).

%%
data_vis_publish(Object) :-
  ros_publish('/data_vis_msgs', 'data_vis_msgs/DataVis', Object).

%% data_vis_remove(+Identifier) is det.
%
% Republishes a data_vis object with empty data
% so that visualization clients can remove it.
%
data_vis_remove(Identifier) :-
  data_vis_(Identifier,0,Object),
  data_vis_publish(Object).
