
%	Plugins
setting(knowrob:plugins,
	[ 'ros/tf/tf_plugin',
	  'ros/marker/marker_plugin'
	]).

%	Triple DB
setting(tripledb:path, 'db/mongo/tripledb/plugin').
setting(tripledb:drop_graphs, [user]). % erase "user" graph on start-up

%	Mongo DB
setting(mng_client:db_name, roslog).

% Flag for read only mongo databases
setting(mng_client:read_only, false).

%	Remote Mongo Setting
setting(mng_client:collection_prefix, '').


