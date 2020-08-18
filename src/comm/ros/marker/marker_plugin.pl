:- module(marker_plugin,
	[ show_marker/2,
	  show_marker/3,
	  hide_marker/1,
	  marker_type/2,
	  marker_action/2
	]).

%%%%%%%%%%%%%%%%
%% TODO
% - trajectory marker
%    -> use points/lines primitive?
% - hook to shows predicate
%

:- use_module(library(settings)).
:- use_module(library('db/scope'),
    [ current_scope/1
    ]).
:- use_module('object_marker').
:- use_foreign_library('libmarker_plugin.so').

:- multifile marker_factory/3.

% define some settings
:- setting(auto, boolean, true,
	'Toggle whether marker messages are generated automatically when an object changes.').

%%
:- message_queue_create(_,[alias(ros_marker_queue)]).

%% marker_action(?ActionTerm,?ActionID) is det.
%
% Maps action term to type id.
%
% @param ActionTerm marker action term
% @param ActionID marker action id
%
marker_action(add,       0).
marker_action(modify,    1). % NOTE: deprecated
marker_action(delete,    2).
marker_action(deleteall, 3).

%% marker_type(?TypeTerm,?TypeID) is det.
%
% Maps type term to type id.
%
% @param TypeTerm marker type term
% @param TypeID marker type id
%
marker_type(arrow,            0).
marker_type(cube,             1).
marker_type(sphere,           2).
marker_type(cylinder,         3).
marker_type(line_strip,       4).
marker_type(line_list,        5).
marker_type(cube_list,        6).
marker_type(sphere_list,      7).
marker_type(points,           8).
marker_type(text_view_facing, 9).
marker_type(mesh_resource,    10).
marker_type(triangle_list,    11).

%% show_marker(+MarkerID,+MarkerTerm) is semidet.
%
% Same as show_marker/2 with empty options list.
%
% @param MarkerID marker id
% @param MarkerTerm marker term
%
show_marker(MarkerID, MarkerTerm) :-
	show_marker(MarkerID, MarkerTerm, []).

%% show_marker(+MarkerID,+MarkerTerm,+Options) is semidet.
%
% Add a visualization marker.
%
% @param MarkerID marker id
% @param MarkerTerm marker term
% @param Options marker options
%
show_marker(MarkerID, MarkerTerm, Options) :-
	% TODO: validate MarkerTerm
	thread_send_message(
		ros_marker_queue,
		marker(add, MarkerID, MarkerTerm, Options)
	).

%% hide_marker(+MarkerID) is det.
%
% Remove a visualization marker.
%
% @param MarkerID marker id
%
hide_marker(MarkerID) :-
	thread_send_message(
		ros_marker_queue,
		marker(delete, MarkerID, _, [])
	).

%%
% Get marker data as Prolog list:
%   [Action,ID,Type,Pose,Scale,Color,Mesh,Text]
%
marker_message(marker(add,ID,Term,Parameters),
		[Action,ID0,Type,Pose,Scale,Color,Mesh,Text]) :-
	%% get marker data
	get_marker_scope(Parameters,Scope),
	marker_message1(Term,[[],Scope]->_,ID->ID0,Data0),
	%% overwrite parameters
	merge_options(Parameters,Data0,Data),
	%%
	option(type(TypeTerm),Data),
	option(pose(Pose),Data),
	option(scale(Scale),Data,[1,1,1]),
	option(color(Color),Data,[1,1,1,1]),
	option(mesh(Mesh),Data,''),
	option(text(Text),Data,''),
	%%
	marker_action(add,Action),
	marker_type(TypeTerm,Type).

marker_message(marker(modify,ID,Term,Opts),Msg) :-
	marker_message(marker(add,ID,Term,Opts),Msg),
	!.

marker_message(marker(delete_all,_,_,_),[Action]) :-
	marker_action(delete_all,Action),
	!.

marker_message(marker(delete,ID,_,_),[Action,ID]) :-
	marker_action(delete,Action),
	!.

%%
marker_message1(MarkerData,_,ID->ID,MarkerData) :-
	is_list(MarkerData),
	!.

marker_message1(Object,Scope,_->ID,MarkerData) :-
	atom(Object),
	object_marker(Object,Scope,ID,MarkerData).

marker_message1(MarkerTerm,Scope,ID->ID,MarkerData) :-
	compound(MarkerTerm),
	marker_factory(MarkerTerm,Scope,MarkerData),
	!.

%%
get_marker_scope(Options,Scope) :-
	option(scope(Scope),Options),
	!.

get_marker_scope(Options,Scope) :-
	option(time(Stamp),Options),
	time_scope(=<(Stamp), >=(Stamp), Scope),
	!.

get_marker_scope(_,Scope) :-
	current_scope(Scope).

%%
% Get all queued markers.
%
marker_pull_all([Head|Rest],Opts) :-
	thread_get_message(
		ros_marker_queue,
		Head,
		Opts),
	!,
	marker_pull_all(Rest,[timeout(0)]).

marker_pull_all([], _).

%%
% Delete redundant markers.
%
marker_to_set([marker(_,ID,_,_)|Xs],Ys) :-
	% ignore in case another marker with same ID is in Xs
	member(marker(_,ID,_,_),Xs),
	!,
	marker_to_set(Xs,Ys).

marker_to_set([X|Xs],[X|Ys]) :-
	!,
	marker_to_set(Xs,Ys).

marker_to_set([],[]).

%%
% Loop and publish queued markers.
%
marker_loop :-
	repeat,
	%%
	marker_pull_all(MarkerTerms,[]),
	marker_to_set(MarkerTerms,MarkerTerms0),
	%%
	setof(MarkerMessage,
		(	member(MarkerTerm,MarkerTerms0),
			marker_message(MarkerTerm,MarkerMessage)
		),
		MessageList),
	marker_array_publish(MessageList),
	%%
	fail.

%%
% Start thread when this file is consulted.
%
:- thread_create(marker_loop,_).

%%
% notify_hook is called whenever an object changes.
%
notify:notify_hook(object_changed(Obj)) :-
	( setting(marker_plugin:auto,true)
	-> show_marker(Obj, Obj, [])
	;  true
	).
