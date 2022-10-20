:- module(knowrob,
	[ knowrob_load_plugins/0,
	  knowrob_load_neem/1
	]).

:- use_module(library(settings)).
:- use_module(library('lang/subgraph')).

% define some settings
:- setting(plugins, list, [], 'List of auto-loaded plugins').

%%
%
%
initialize_db :-
	% initialize hierachical organization of triple graphs
	add_subgraph(user,common),
	add_subgraph(test,user),
	load_graph_structure.
	

%% knowrob_load_neem(+NEEM_id) is det.
%
% Configure KnowRob to use the DB associated to some NEEM,
% and initialize position data etc.
%
knowrob_load_neem(NEEM_id) :-
	% assign DB collection prefix
	set_setting(mng_client:collection_prefix, NEEM_id),
	mng_get_db(_DB, CollectionName, 'tf'),
	mongolog_update_fluent_options('tf_raw',[collection(CollectionName)]),
	% re-initialize the triple DB
	% this is important e.g. to establish triple graph hierarchy.
	% else we may get orphaned graphs.
	load_graph_structure,
	% load URDF files referred to in triple store
	urdf_init,
	% initialize position of each frame for tf publishing
	tf:tf_republish_clear,
	tf_mongo:tf_mng_lookup_all(InitialTransforms),
	forall(
	    (   member([Ref,Frame,Pos,Rot],InitialTransforms),
	        % FIXME avoid this elsewhere
	        Ref \= Frame,
	        \+ atom_concat('/',Ref,Frame),
	        \+ atom_concat('/',Frame,Ref)
	    ),
		tf:tf_republish_set_pose(Frame,[Ref,Pos,Rot])
	),
	% publish object marker messages
	marker:republish.

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
