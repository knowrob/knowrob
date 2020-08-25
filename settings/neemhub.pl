
%	Plugins
setting(knowrob:plugins,
	[ 'comm/ros/tf/tf_plugin',
	  'comm/ros/marker/marker_plugin'
	]).

%	Triple DB
setting(tripledb:path, 'db/mongo/tripledb/plugin').
setting(tripledb:drop_graphs, []). % erase "user" graph on start-up

% Flag for read only mongo databases
setting(mng_client:tripledb_read_only, true).

%	Neemhub Setting
setting(mng_client:db_name, 'neems').
setting(mng_client:neemhub_neem_id, '5f22b1f512db5aed7cd1961b').

