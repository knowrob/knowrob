:- module(knowrob,
	[ knowrob_load_settings/0,
	  knowrob_load_plugins/0,
	  neem_init0/1
	]).

:- use_module(library(settings)).

% define some settings
:- setting(plugins, list, [], 'List of auto-loaded plugins').

neem_init0(NEEM_id) :-
	% assign DB collection prefix
	set_setting(mng_client:collection_prefix, NEEM_id),
	% load URDF files referred to in triple store
	urdf_init,
	% initialize position of each frame for tf publishing
	tf_tree:initial_transforms(InitialTransforms),
	forall(
	    (   member([Ref,Frame,Pos,Rot],InitialTransforms),
	        % FIXME avoid this elsewhere
	        Ref \= Frame,
	        \+ atom_concat('/',Ref,Frame),
	        \+ atom_concat('/',Frame,Ref)
	    ),
		tf_plugin:tf_republish_set_pose(Frame,[Ref,Pos,Rot])
	),
	% publish object marker messages
	marker_plugin:republish.

%% restore_settings is det.
%
% Load settings from file.
%
knowrob_load_settings :-
	( getenv('KNOWROB_SETTINGS', File)
	-> load_settings(File,[undefined(load)])
	;  true
	).

%% load_plugins is det.
%
% Load plugins that are activated in settings.
%
knowrob_load_plugins :-
	setting(knowrob:plugins,Plugins),
	forall(
		member(Plugin,Plugins),
		knowrob_load_plugin(Plugin)
	).

knowrob_load_plugin(Plugin) :-
	ensure_loaded(library(Plugin)).
