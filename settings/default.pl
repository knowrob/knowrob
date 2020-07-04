
%	Plugins
setting(knowrob:plugins,
	[ 'comm/ros/tf/tf_plugin',
	  'comm/ros/marker/marker_plugin'
	]).

%	Triple DB
setting(tripledb:path,  'db/mongo/tripledb').
setting(tripledb:whipe, yes).

%	Mongo DB
setting(mng_client:db_name, roslog).
