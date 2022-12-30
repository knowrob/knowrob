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
      timeline/1,
      timeline_data/1,
      timeline_data/2
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
%    data_vis(piechart(chart_id), [
%      title: 'Some Distribution',
%      data: [[a,b,c],[10,30,22]]
%    ]).
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
      is_classified_by(Evt, Task),
      rdf_split_url(_,EvtName,Task))),
    time_interval_data(Evt, _,_)
  ), EvtNames),
  findall(Time, (
    member(Evt, Events),
    once((
      time_interval_data(Evt, T0,T1),
      atomic_list_concat([T0,T1], '_', Time)))
  ), EventExtends),
  data_vis(timeline(event_timeline),
          [values:[EvtNames,EventExtends]]).

%% timeline_data(+Events:list) is det
%
% Creates a new data_vis timeline message and publishes it via _|/data_vis_msgs|_
% ROS topic.
%
% @param EventData list of list of the form [[Events, Task, Start, End]]
%
timeline_data(EventsData) :-
	timeline_data(EventsData,[]).

timeline_data(EventsData,Options) :-
  findall(EvtName, (
    member([_,Task,_,_],EventsData),
    once((
      rdf_split_url(_,EvtName,Task)))
  ), EvtNames),
  findall(Time, (
    member([_,_,Start,End], EventsData),
    atomic_list_concat([Start, End],'_',Time)
  ), EventExtends),
  data_vis(timeline(event_timeline),
          [values:[EvtNames,EventExtends] | Options]).

data_vis_set_(Key,Msg,Value) :-
  get_dict(Key,Msg,[Type,_]),
  b_set_dict(Key,Msg,[Type,Value]).

%% data_vis_object
data_vis_object(piechart(Identifier), Object) :-
  data_vis_(Identifier,0,Object).
data_vis_object(barchart(Identifier), Object) :-
  data_vis_(Identifier,1,Object).
data_vis_object(treechart(Identifier), Object) :-
  data_vis_(Identifier,2,Object).
data_vis_object(timeline(Identifier), Object) :-
  data_vis_(Identifier,3,Object).
data_vis_object(linechart(Identifier), Object) :-
  data_vis_(Identifier,4,Object).
data_vis_object(table(Identifier), Object) :-
  data_vis_(Identifier,100,Object).
data_vis_object(graph(Identifier), Object) :-
  data_vis_(Identifier,999,Object).

data_vis_object(type(Identifier,TypeID), Object) :-
  data_vis_(Identifier,TypeID,Object).

data_vis_(Identitier, Type, _{
  id:       [string,Identitier],
  type:     [int32,Type],
  title:    [string,''],
  xlabel:   [string,''],
  ylabel:   [string,''],
  width:    [int32,200],
  height:   [int32,200],
  fontsize: [string,'12px'],
  values:   ['array(data_vis_msgs/ValueList)',[]]
}).

%% data_vis_set_data
data_vis_set_data(Object, Data) :-
  data_vis_set_property(Object, values:Data).

%% data_vis_set_property
data_vis_set_property(Object, data:Data) :-
  data_vis_set_property(Object, values:Data),!.
data_vis_set_property(Object, values:Data) :-
  data_vis_values(Data,Values),!,
  data_vis_set_(values, Object, [Values]).
data_vis_set_property(Object, array_data:Array) :-
	!,
	findall(Y,
		(	member(X,Array),
			data_vis_values(X,Y)
		),
		ValuesArray
	),
	data_vis_set_(values, Object, ValuesArray).
data_vis_set_property(Object, Key:Value) :-
  data_vis_set_(Key, Object, Value).

%% data_vis_set_properties
data_vis_set_properties(_, []) :- !.
data_vis_set_properties(Object, [Prop|Rest]) :-
  data_vis_set_property(Object, Prop), !,
  data_vis_set_properties(Object, Rest).

%%
data_vis_values([Data1,Data2],_{
  label: [string,''],
  value1: ['array(string)',D1],
  value2: ['array(string)',D2]
}) :-
  string_data_(Data1,D1),
  string_data_(Data2,D2),!.
data_vis_values(Data1,_{
  label: [string,''],
  value1: ['array(string)',D1],
  value2: ['array(string)',[]]
}) :-
  string_data_(Data1,D1),!.

%%
string_data_([],[]) :- !.
string_data_([X|Xs],[Y|Ys]) :-
  ( atom(X) -> Y=X ; term_to_atom(X,Y) ),
  string_data_(Xs,Ys).

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
