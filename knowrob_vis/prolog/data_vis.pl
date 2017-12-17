/** <module> Methods for visualizing parts of the knowledge base

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

  @author Moritz Tenorth
  @license BSD
*/

:- module(data_vis,
    [
      diagram_canvas/0,
      clear_diagram_canvas/0,
      add_diagram/9,
      add_diagram/10,
      add_barchart/3,
      add_piechart/3,
      add_timeline/5,
      remove_diagram/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).


:- rdf_meta add_diagram(+,+,+,+,+,+,+,+,+),
            remove_diagram(+).

:- assert(d_canvas(fail)).
%diagram_canvas :-
%    d_canvas(fail),
%    jpl_new('org.knowrob.vis.DiagramVisualization', [], Canvas),
%    retract(d_canvas(fail)),
%    assert(d_canvas(Canvas)),!.
%diagram_canvas(Canvas) :-
%    d_canvas(Canvas).

diagram_canvas :-
    d_canvas(fail),
    jpl_new('org.knowrob.vis.DiagramVisualization', [], Canvas),
    jpl_list_to_array(['org.knowrob.vis.DiagramVisualization'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Canvas, Arr], _),
    retract(d_canvas(fail)),
    assert(d_canvas(Canvas)),!.
diagram_canvas(Canvas) :-
    d_canvas(Canvas).

:- diagram_canvas.


%% add_diagram(+Id, +Title, +Type, +Xlabel, +Ylabel, +Width, +Height, +Fontsize, +ValueList) is nondet.
%
% Add object to the scene with its position at time 'Time'
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param Type       Type of the diagram (piechart, barchart or treechart)
% @param Xlabel     Label for the X axis
% @param Ylabel     Label for the Y axis
% @param Width      Width of the diagram in px
% @param Height     Height of the diagram in px
% @param Fontsize   Fontsize for the labels
% @param RowLabels  List of labels for the data rows (optional)
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, ValueList) :-
  add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, @(null), ValueList).
  
add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, RowLabels, ValueList) :-
  once((
    d_canvas(Canvas),
    lists_to_arrays(ValueList, ValueArr),
    once((lists_to_arrays(RowLabels, RowLabelArr) ; RowLabelArr = @(null))),
    jpl_call(Canvas, 'addDiagram', [Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, RowLabelArr, ValueArr], _)
  )).


%% help_timeline(+SVal, +EVal, -Res)
%
% Helpfunction for add_timeline that concatenates Start and End time to one atom so it can be passed to add_diagram in the same fashion as the other data types
help_timeline(SVal, EVal, Res) :-
  atom_concat(SVal, '_', Tmp),
  atom_concat(Tmp, EVal, Res).

%% add_timeline(+Id, +Title, +EventsList, +StartList, +EndList) is nondet.
%
% Simplified predicate for adding a timeline with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_timeline(Id, Title, EventsList, StartList, EndList) :-
  %Concatenate Start and endtimes so they can be given to add_diagram as one value
  maplist(help_timeline,StartList,EndList, TimeList),
  %Call add_diagram with the appropriate settings
  add_diagram(Id, Title, 'timeline', 'Time', 'Events', 250, 250, '12px', [[EventsList,TimeList]]).

%% add_piechart(+Id, +Title, +ValueList) is nondet.
%
% Simplified predicate for adding a piechart with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_piechart(Id, Title, ValueList) :-
    add_diagram(Id, Title, 'piechart', '', '', 250, 250, '9px', ValueList).


%% add_barchart(+Id, +Title, +ValueList) is nondet.
%
% Simplified predicate for adding a barchart with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_barchart(Id, Title, ValueList) :-
    add_diagram(Id, Title, 'barchart', '', '', 250, 250, '9px', ValueList).

    
%% remove_diagram(+Id) is det.
%
% Remove the diagram specified by 'id'
%
% @param Id Unique string identifier for the chart to be removed
% 
remove_diagram(Id) :-
    d_canvas(Canvas),
    jpl_call(Canvas, 'removeDiagram', [Id], _),!.

%% clear_diagram_canvas is det.
%
% Remove all diagrams from the canvas.
% 
clear_diagram_canvas :-
    d_canvas(Canvas),
    jpl_call(Canvas, 'clear', [], _).
