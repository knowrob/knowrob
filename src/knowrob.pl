:- module(knowrob,
	[ knowrob_load_settings/0,
	  knowrob_load_plugins/0
	]).

:- use_module(library(settings)).

% define some settings
:- setting(plugins, list, [], 'List of auto-loaded plugins').

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
