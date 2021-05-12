:- module(marker,
	[ show_marker/2,
	  show_marker/3,
	  show_markers/1,
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
:- use_module(library('lang/scope'),
    [ current_scope/1 ]).
:- use_module('object_marker').

:- use_foreign_library('libmarker_knowrob.so').

:- multifile marker_factory/3.

% define some settings
%:- setting(auto, boolean, true,
%	'Toggle whether marker messages are generated automatically when an object changes.').
%:- setting(reference_frame, atom, '',
%	'The reference frame in which the position of markers is published.').

%%
%:- message_queue_create(_,[alias(ros_marker_queue)]).

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
show_marker(_MarkerID, _MarkerTerm, _Options) :-
	writeln('WARNING show_marker called'),
	fail.
	% TODO: validate MarkerTerm
%	thread_send_message(
%		ros_marker_queue,
%		marker(add, MarkerID, MarkerTerm, Options)
%	).


%% show_marker(+TimePoint) is semidet.
%
% Republish all markers of a given timepoint
%
% @param Timepoint The given timepoint
%
show_markers(Timepoint) :-
	%time_scope(=<(Timepoint), >=(Timepoint), Scope),
	%forall(
	%	kb_call(is_physical_object(PO)),
	%	show_marker(PO, PO, [scope(Scope)])
	%).
	findall(Msg,
		(	object_marker(_Obj,ID,Data),
			marker_message_new(ID,Data,Msg)
		),
		MessageList
	),
	(	MessageList=[] -> true
	;	marker_array_publish(MessageList)
	).

%%
% Republish all object markers.
%
republish :-
	get_time(Now),
	show_markers(Now).


%% hide_marker(+MarkerID) is det.
%
% Remove a visualization marker.
%
% @param MarkerID marker id
%
hide_marker(_MarkerID) :-
	writeln('WARNING hide_marker called'),
	fail.
%	thread_send_message(
%		ros_marker_queue,
%		marker(delete, MarkerID, _, [])
%	).

%%
marker_message_new(MarkerID,Data,
		[Action,MarkerID,Type,Pose,Scale,Color,Mesh,Text]) :-
	!,
	option(type(TypeTerm),Data),
	option(pose(Pose),Data),
	option(scale(Scale),Data,[1,1,1]),
	option(color(Color),Data,[1,1,1,1]),
	option(mesh(Mesh),Data,''),
	option(text(Text),Data,''),
	%%
	marker_action(add,Action),
	marker_type(TypeTerm,Type).

%%
% Get marker data as Prolog list:
%   [Action,ID,Type,Pose,Scale,Color,Mesh,Text]
%
marker_message(marker(add,ID,Term,Parameters),Msg) :-
	!,
	%% get marker data
	get_marker_scope(Parameters,Scope),
	marker_message1(Term,[[],Scope]->_,ID->ID0,Data0),
	%% overwrite parameters
	merge_options(Parameters,Data0,Data),
	%%
	marker_message_new(ID0,Data,Msg).

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
marker_message1(Object,Scope,_->ID,MarkerData) :-
	atom(Object),
	!,
	object_marker(Object,Scope,ID,MarkerData).

marker_message1(MarkerData,_,ID->ID,MarkerData) :-
	is_list(MarkerData),
	!.

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
	current_scope(Scope),
	!.

%%
% Get all queued markers.
%
%marker_pull_all([Head|Rest],Opts) :-
%	thread_get_message(
%		ros_marker_queue,
%		Head,
%		Opts),
%	!,
%	marker_pull_all(Rest,[timeout(0)]).
%
%marker_pull_all([], _).

%%
% Delete redundant markers.
%
%marker_to_set([marker(_,ID,_,_)|Xs],Ys) :-
%	% ignore in case another marker with same ID is in Xs
%	member(marker(_,ID,_,_),Xs),
%	!,
%	marker_to_set(Xs,Ys).
%
%marker_to_set([X|Xs],[X|Ys]) :-
%	!,
%	marker_to_set(Xs,Ys).
%
%marker_to_set([],[]).

%%
% Loop and publish queued markers.
%
%marker_loop :-
%	repeat,
%	%%
%	log_info(loop0),
%	marker_pull_all(MarkerTerms,[]),
%	log_info(loop1),
%	get_time(T0),
%	marker_to_set(MarkerTerms,MarkerTerms0),
%	findall(MarkerMessage,
%		(	member(MarkerTerm,MarkerTerms0),
%			marker_message(MarkerTerm,MarkerMessage)
%		),
%		MessageList),
%	(	MessageList=[] -> true
%	;	marker_array_publish(MessageList)
%	),
%	get_time(T1),
%	length(MarkerTerms0,Count),
%	Took is T1 - T0,
%	log_info(looped(Took,Count)),
%	%%
%	fail.

%%
% Start thread when this file is consulted.
%
%:- thread_create(marker_loop,_).

%%
% notify_hook is called whenever an object changes.
%
%notify:notify_hook(object_changed(Obj)) :-
%	( setting(marker:auto,true)
%	-> show_marker(Obj, Obj, [])
%	;  true
%	).
