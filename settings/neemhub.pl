
%	Plugins
setting(knowrob:plugins,
	[ 'ros/tf/tf_plugin',
	  'ros/marker/marker_plugin'
	]).

setting(marker_plugin:auto, false).
setting(marker_plugin:reference_frame, 'map').

%	Triple DB
setting(tripledb:path, 'db/mongo/tripledb/plugin').
setting(tripledb:drop_graphs, []).

% Flag for read only mongo databases
setting(mng_client:read_only, true).

%	Remote Mongo Setting
setting(mng_client:db_name, 'neems').
setting(mng_client:collection_prefix, '5f22b1f512db5aed7cd1961a').


