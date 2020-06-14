:- module(mng_client,
    [ mng_db_name/1,
      mng_collection/2,
      mng_distinct_values/4,
      mng_drop/2,
      mng_store/3,
      mng_update/4,
      mng_remove/3,
      mng_index_create/2,
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
      mng_pl_value/2,
      mng_get_dict/3
    ]).
/** <module> A mongo DB client for Prolog.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).

:- use_foreign_library('libmongo_kb.so').

read_json_(JSON,Dict) :-
  atom_to_chars(JSON,Chars),
  open_chars_stream(Chars,Stream),
  json_read_dict(Stream,Dict).

%%
mng_db_name(DBName) :-
  getenv('KNOWROB_MONGO_DB_NAME',DBName),
  \+ DBName='', !.
mng_db_name(roslog).

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
      mng_pl_value(V_mng,V)
  ), DistinctValues).

%%
mng_index_create(DB,Indices) :-
  forall(
    ( member([Coll,Keys],Indices),
      member(Key,Keys) ),
    ( mng_index_create(DB,Coll,Key) )
  ).
  
  
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
  mng_cursor_next_json(Cursor,JSON),
  read_json_(JSON,Dict).

%% 
mng_cursor_materialize(Cursor,Dict) :-
  mng_cursor_next(Cursor,Next),
  mng_cursor_materialize(Cursor,Next,Dict).

mng_cursor_materialize(Cursor,Next,Dict) :-
  ( mng_cursor_next(Cursor,Next1) ->
    ( Dict=Next ; mng_cursor_materialize(Cursor,Next1,Dict) );
    ( Dict=Next )
  ).

%% mng_get_dict(?Key,+Doc,?PlValue) is semidet.
%
% Get a key-value pair from a dictionary, and map
% any data value to native Prolog types using mng_pl_value/2.
%
mng_get_dict(Key,Doc,PlValue) :-
  get_dict(Key,Doc,MngValue),
  mng_pl_value(MngValue,PlValue).

%% mng_pl_value(+ValueMongo,?ValueNative) is det.
%
% Query results are mapped into Prolog dictionaries,
% with some particularities typed data is represented.
% This predicate is used to eliminate type annotations
% while mapping data values to appropiate native
% Prolog types.
%
mng_pl_value(Dict,Val_pl) :-
  is_dict(Dict),!,
  % peek element, in case the key starts with '$',
  % read as atomic value
  once(get_dict(Type,Dict,Val)),
  ( atom_concat('$',_,Type) ->
    mng_pl_dict_value_(Type,Val,Val_pl) ;
    mng_pl_value_dict(Dict,Val_pl)
  ).

mng_pl_value(List,array(List0)) :-
  is_list(List),!,
  findall(X,(
    member(Y,List),
    mng_pl_value(Y,X)
  ),List0).

mng_pl_value(String,string(Val)) :-
  string(String),
  string_to_atom(String,Val),!.

%mng_pl_value(String,string(Val)) :-
  %string(String),
  %string_to_atom(String,A),
  %( catch(term_to_atom(Val,A),_,Val=A) ),
  %( ground(Val);Val=A ),!.

mng_pl_value(Val,Val).

mng_pl_value_dict(In,Out) :-
  findall(K-V_pl, (
    get_dict(K,In,V_mng),
    mng_pl_value(V_mng,V_pl)
  ),Pairs),
  dict_pairs(Out,_,Pairs).

%%
mng_pl_dict_value_('$numberInt',Val,int(Val)) :-
  number(Val), !.
mng_pl_dict_value_('$numberInt',String,int(Val)) :-
  string(String),!,
  parse_number_(String,Val).
mng_pl_dict_value_('$numberLong',Val,int(Val)) :-
  number(Val), !.
mng_pl_dict_value_('$numberLong',String,int(Val)) :-
  string(String),!,
  parse_number_(String,Val).
mng_pl_dict_value_('$numberDecimal',Val,double(Val)) :- number(Val), !.
mng_pl_dict_value_('$numberDecimal','Infinity',double('Infinity')) :- !.
mng_pl_dict_value_('$numberDecimal',"Infinity",double('Infinity')) :- !.
mng_pl_dict_value_('$numberDecimal',String,double(Val)) :-
  string(String),!,
  parse_number_(String,Val).
mng_pl_dict_value_('$numberDouble',Val,double(Val)) :-
  number(Val), !.
mng_pl_dict_value_('$numberDouble',String,double(Val)) :-
  string(String),!,
  parse_number_(String,Val).
mng_pl_dict_value_('$date',DateDict,double(Time)) :-
  mng_pl_value(DateDict,int(Long)),
  Time is Long/1000.0.
mng_pl_dict_value_('$oid',Val,id(Val)) :-
  atom(Val), !.
mng_pl_dict_value_('$oid',Val,id(Atom)) :-
  string(Val), !,
  string_to_atom(Val,Atom).
mng_pl_dict_value_(Type,Val,Val) :-
  write('Warn: unknown mng type: '), write([Type,Val]), nl.

%%
parse_number_(String,Number) :-
  string_to_atom(String,Atom),
  atomic_list_concat([A0,A1],',',Atom),
  atomic_list_concat([A0,A1],'.',X),
  term_to_atom(Number,X),!.
parse_number_(String,Number) :-
  number_string(Number,String).

%% mng_dump(+DB, +Dir) is det.
%
% Dump mongo DB.
%
mng_dump(DB,Dir) :-
  process_create(path(mongodump),
    [ '--db', DB, '--out', Dir ],
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
  process_create(path(mongorestore),
    [ Dir ],
    [ process(PID) ]
  ),
  wait(PID,exited(0)).


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

%% mng_index_create(+DB,+Collection,+Keys) is det
%
% Creates search index.
%
% @param DB The database name
% @param Collection The collection name
% @param Keys List of keys for which an index shall be created
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
