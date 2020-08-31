
%	Plugins
setting(knowrob:plugins,
	[ 'comm/ros/tf/tf_plugin',
	  'comm/ros/marker/marker_plugin'
	]).

%	Triple DB
setting(tripledb:path, 'db/mongo/tripledb/plugin').
setting(tripledb:drop_graphs, [user]). % erase "user" graph on start-up

%	Mongo DB
setting(mng_client:db_name, roslog).

% Flag for read only mongo databases
setting(mng_client:tripledb_read_only, false).

%	Neemhub Setting
% setting(mng_client:db_name, 'neems').
setting(mng_client:neemhub_neem_id, '').


