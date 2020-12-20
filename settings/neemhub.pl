
%	Plugins
setting(knowrob:plugins,
	[ 'ros/tf/tf_plugin',
	  'ros/marker/marker_plugin'
	]).

setting(marker_plugin:auto, false).
%setting(marker_plugin:reference_frame, 'map').

setting(tf_plugin:use_logger, false).

%	Triple DB
setting(tripledb:path, 'db/mongo/tripledb/plugin').
setting(tripledb:drop_graphs, []).

% Flag for read only mongo databases
setting(mng_client:read_only, true).

%	Remote Mongo Setting
setting(mng_client:db_name, 'neems').
setting(mng_client:collection_prefix, '5fc8ff968f880006aa208e19').


