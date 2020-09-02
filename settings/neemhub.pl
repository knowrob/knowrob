
%	Plugins
setting(knowrob:plugins,
	[ 'comm/ros/tf/tf_plugin',
	  'comm/ros/marker/marker_plugin'
	]).

%	Triple DB
setting(tripledb:path, 'db/mongo/tripledb/plugin').
setting(tripledb:drop_graphs, []). % erase "user" graph on start-up

% Flag for read only mongo databases
setting(mng_client:read_only, true).

%	Remote Mongo Setting
setting(mng_client:db_name, 'neems').
setting(mng_client:collection_prefix, '5f22b1f512db5aed7cd1961b').


