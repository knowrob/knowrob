:- module(mng_client,
    [ mng_db_name/1,
      mng_get_db/3,
      mng_collection/2,
      mng_distinct_values/4,
      mng_drop/2,
      mng_store/3,
      mng_update/4,
      mng_remove/3,
      mng_index_create/2,
      mng_index_create/3,
      mng_export/1,
      mng_export_collection/2,
      mng_dump/2,
      mng_dump_collection/3,
      mng_restore/2,
      mng_regex_prefix/2,
      mng_cursor_create/3,
      mng_cursor_destroy/1,
      mng_cursor_filter/2,
      mng_cursor_descending/2,
      mng_cursor_ascending/2,
      mng_cursor_limit/2,
      mng_cursor_next/2,
      mng_cursor_materialize/2,
      mng_get_dict/3,
      mng_uri/1
    ]).
/** <module> A mongo DB client for Prolog.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).
:- use_foreign_library('libmongo_kb.so').
:- dynamic mng_db_name/1.

% define some settings
:- setting(db_name, atom, roslog,
		'Name of the Mongo DB used by KnowRob.').
:- setting(mng_client:collection_prefix, atom, '',
    'Id of the current neem. Empty if neemhub is not used').
:- setting(mng_client:read_only, atom, false,
    'Flag if the tripledb is read only').

:- setting(mng_client:db_name, DBName),
   assertz(mng_db_name(DBName)).



%% mng_get_db(?DB, -CollectionName, +DBType) is det.
%
% Get db and collection for this type 
% of data, e.g. triples
%
% @param DB The database name
% @param CollectionNAme The Name of the collection for the type
% @param DBType
%
mng_get_db(DB, CollectionName, DBType) :- 
  mng_db_name(DB),
  ((setting(mng_client:collection_prefix, Id),Id \= '') 
    -> atomic_list_concat([Id,'_',DBType], CollectionName)
    ; CollectionName = DBType
  ).

read_json_(JSON,Dict) :-
  atom_to_chars(JSON,Chars),
  open_chars_stream(Chars,Stream),
  json_read_dict(Stream,Dict).

%%
%mng_db_name(DBName) :-
%	setting(mng_client:db_name, DBName).

%%
mng_export(Dir) :-
  mng_db_name(DB),
  mng_dump(DB,Dir).

%%
mng_export_collection(Collection,Dir) :-
  mng_db_name(DB),
  mng_dump_collection(DB,Collection,Dir).

%%
mng_drop(DB,Coll) :-
  catch(
    mng_drop_unsafe(DB,Coll),
    % collection does not exist yet
    mng_error(drop_failed('ns not found')),
    true
  ).

%% mng_collection(+DB,?Collection) is det
%
% True iff *Collection* is an existing collection
% in the current DB.
%
mng_collection(DB,Collection) :-
  mng_collections(DB,Collections),
  member(Collection,Collections).

%%
mng_distinct_values(DB,Collection,Key,DistinctValues) :-
  mng_distinct_values_json(DB,Collection,Key,DistinctJSON),
  read_json_(DistinctJSON, DistinctDict),
  get_dict(values, DistinctDict, ValuesMng),
  findall(V, (
      member(V_mng,ValuesMng),
      mng_doc_value(V_mng,V)
  ), DistinctValues).
  
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % regex

%% mng_regex_prefix(+Prefix,-Pattern) is det.
%
% Create a regex pattern for matching entries
% with some prefix.
%
% @param Prefix an atom
% @param Pattern regex pattern for matching the prefix of values
%
mng_regex_prefix(Prefix,Pattern) :-
	atom_codes(Prefix, AtomCodes),
	mng_regex_prefix_(AtomCodes,PatternCodes),
	atom_codes(Pattern0, PatternCodes),
	atomic_list_concat(['^',Pattern0,'.*'], '', Pattern).

%%
mng_regex_prefix_([],[]) :- !.

mng_regex_prefix_([X|Xs],[X|Ys]) :-
	char_type(X,alnum),!,
	mng_regex_prefix_(Xs,Ys).

mng_regex_prefix_([X|Xs],[BS,X|Ys]) :-
	char_code('\\', BS),
	mng_regex_prefix_(Xs,Ys).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Query Cursor

%% mng_cursor_next(+Cursor,?Dict) is semidet.
%
%
mng_cursor_next(Cursor,Dict) :-
  mng_cursor_next_pairs(Cursor,Pairs),
  dict_pairs(Dict,_,Pairs).

%% 
%mng_cursor_materialize(Cursor,Next) :-
%	(	mng_cursor_next(Cursor,X)
%	->	(	Next=X
%		;	mng_cursor_materialize(Cursor,Next)
%		)
%	;	fail
%	).

mng_cursor_materialize(Cursor,Dict) :-
  mng_cursor_next(Cursor,Next),
  mng_cursor_materialize(Cursor,Next,Dict).

mng_cursor_materialize(Cursor,Next,Dict) :-
  ( mng_cursor_next(Cursor,Next1) ->
    ( Dict=Next ; mng_cursor_materialize(Cursor,Next1,Dict) );
    ( Dict=Next )
  ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Query Index

%% mng_index_create(+DB,+Collection,+Keys) is det
%
% Creates search index.
%
% @param DB The database name
% @param Collection The collection name
% @param Keys List of keys for which an index shall be created
%
mng_index_create(DB,Collection,Keys) :-
	findall(K,
		(	member(K0,Keys),
			format_key_(K0,K)
		),
		Keys0
	),
  ( setting(mng_client:read_only, true)
    -> true
    ; mng_index_create_core(DB,Collection,Keys0)
  ).

%%
mng_index_create(DB,Indices) :-
  forall(
    member([Coll,Keys],Indices),
    mng_index_create(DB,Coll,Keys)
  ).

%%
format_key_(+(K),+(K)) :- !.
format_key_(-(K),-(K)) :- !.
format_key_(  K, +(K)) :- !.


%% mng_get_dict(?Key,+Doc,?PlValue) is semidet.
%
% Get a key-value pair from a dictionary, and map
% any data value to native Prolog types using mng_pl_value/2.
%
mng_get_dict(Key,Doc,PlValue) :-
  get_dict(Key,Doc,MngValue),
  mng_doc_value(MngValue,PlValue).

%%
mng_doc_value([X|Xs],PlValue) :-
	findall(Key-PlValue_n,
		(	member(Key-MngValue_n,[X|Xs]),
			mng_doc_value(MngValue_n,PlValue_n)
		),
		Pairs
	),
	dict_pairs(PlValue,_,Pairs), !.

mng_doc_value(PlValue,PlValue).

%% mng_dump(+DB, +Dir) is det.
%
% Dump mongo DB.
%
mng_dump(DB,Dir) :-
  mng_uri(URI),
  process_create(path(mongodump),
    [ '--uri', URI, '--db', DB, '--out', Dir ],
    [ process(PID) ]
  ),
  wait(PID,exited(0)).

%% mng_dump_collection(+DB, +Collection, +Dir) is det.
%
% Dump mongo DB.
%
mng_dump_collection(DB,Collection,Dir) :-
  process_create(path(mongodump),
    [ '--db', DB, '--collection', Collection, '--out', Dir ],
    [ process(PID) ]
  ),
  wait(PID,exited(0)).

%% mng_restore(+DB, +Dir) is det.
%
% Restore mongo DB.
%
mng_restore(_DB,Dir) :-
  mng_uri(URI),
  process_create(path(mongorestore),
    [ '--uri', URI, '--dir', Dir ],
    [ process(PID) ]
  ),
  wait(PID,exited(0)).

%% mng_uri(-URI) is det.
%
% Get the URI connection string
%
mng_uri(URI) :-
  getenv('KNOWROB_MONGO_HOST', Host),
  getenv('KNOWROB_MONGO_USER', User),
  getenv('KNOWROB_MONGO_PASS', Pass),
  getenv('KNOWROB_MONGO_PORT', Port),
  atomic_list_concat([ 'mongodb://', User, ':', Pass,
    '@', Host, ':', Port ], URI),
  !.

mng_uri('mongodb://localhost:27017').

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % C++ predicates

%% mng_drop(+DB,+Collection) is det.
%
% Drop a collection.
%
% @param DB The database name
% @param Collection The collection name
%

%% mng_store(+DB, +Collection, +Dict)
%
% Stores a dictionary in a mongo DB collection
% with the provided name.
%
% @param DB The database name
% @param Collection The collection name
% @param Dict A Prolog dictionary
%

%% mng_cursor_create(+DB, +Collection, -Cursor) is det.
%
% Creates a new query cursor.
% Make sure to call *mng_cursor_destroy/1* once
% you are done querying.
%
% @param DB The database name
% @param Collection The collection name
% @param Cursor The id of a new mongo DB cursor
%

%% mng_cursor_destroy(+Cursor) is det.
%
% Destroys a query cursor.
%
% @param Cursor A mongo DB cursor id
%

%% mng_cursor_next_json(+Cursor,-JSON) is det.
%
% Read next document from a query cursor.
%
% @param Cursor A mongo DB cursor id
% @param JSON A JSON-encoded mongo DB document
%

%% mng_cursor_limit(+Cursor, +Limit) is det.
%
% Out is the same mongo DB cursor as In but limited to N results.
%
% @param Cursor A mongo DB cursor id
% @param Limit The maximum number of documents yielded by the cursor
%

%% mng_cursor_descending(+Cursor, +Key) is det.
%
% Sorts the DB cursor In w.r.t. Key, the sorted collection
% can be accessed via the new DB cursor Out.
%
% @param Cursor A mongo DB cursor id
% @param Key The sort key
%

%% mng_cursor_ascending(+Cursor, +Key) is det.
%
% Sorts the DB cursor In w.r.t. Key, the sorted collection
% can be accessed via the new DB cursor Out.
%
% @param Cursor A mongo DB cursor id
% @param Key The sort key
%
