
%	Plugins
setting(knowrob:plugins, []).

%	
setting(marker_plugin:auto, false).

%	Disable logging of TF messages
setting(tf_plugin:use_logger, false).

%	Do not drop any triple graphs on startup
setting(lang_db:drop_graphs, []).

%	Mongo DB name
setting(mng_client:db_name, 'neems').
%	Flag for read only mongo databases
setting(mng_client:read_only, true).
%	A prefix used for all collection names.
setting(mng_client:collection_prefix, '5fc8ff968f880006aa208e19').
