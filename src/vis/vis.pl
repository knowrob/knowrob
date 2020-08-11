/*
  Copyright (C) 2015 Daniel Beßler
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

:- module(knowrob_vis, []).

%:- module(knowrob_vis,
%    [
%      show/1,
%      show/2,
%      hide/1
%    ]).
%/** <module> Methods for visualizing parts of the knowledge base
%
%  @author Daniel Beßler
%  @license BSD
%*/
%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('knowrob/data_vis')).
%
%:- rdf_meta 
%      hide(t),
%      show(t),
%      show(t,t),
%      hide_marker(t),
%      show_marker(t,t).
%
%:- multifile show/2,
%             hide/1,
%             show_marker/2,
%             hide_marker/1.
%
%%% show(+Thing) is det.
%%% show(+Thing, +Properties) is det.
%%
%% This is a non-logical predicate used to invoke
%% external clients such us RViz
%% to visualize objects in the knowledge base.
%% Custom visualization back-ends may be used
%% by defining another clause of this multifile
%% predicate.
%%
%show(Things) :-
%  is_list(Things), !,
%  show_next_,
%  forall( member(Thing, Things), (
%    T =.. [show|Thing], call(T)
%  )), !.
%show(Thing) :-
%  show(Thing,[]).
%show(Thing, Properties) :-
%  show_marker(Thing,Properties).
%
%%% show_next is det
%show_next_ :-
%  true.
%
%%% hide(+Thing) is det.
%%
%% This is a non-logical predicate used to invoke
%% external clients such us RViz
%% to remove visualizations of objects.
%%
%hide(Things) :-
%  is_list(Things), !,
%  forall( member(Thing, Things), (
%    T =.. [hide|Thing], call(T)
%  )), !.
%hide(Thing) :-
%  hide_marker(Thing).
%  
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% % % % % Marker visualization
%
%%%
%show_marker(Thing, Properties) :-
%  atom(Thing),
%  rdf_resource(Thing),
%  object_state(Thing, State, Properties),
%  object_state_add_cpp([State]).
%%%
%hide_marker(Thing) :-
%  atom(Thing),
%  rdf_resource(Thing),
%  object_state_remove_cpp([Thing]).
