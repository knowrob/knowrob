:- module(mng_client,
    [ mng_db_name/1,
      mng_get_db/3,
      mng_one_db/2,
      mng_collection/2,
      mng_distinct_values/4,
      mng_drop/2,
      mng_store/3,
      mng_update/4,
      mng_remove/3,
      mng_find/4,
      mng_index_create/2,
      mng_index_create/3,
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
      mng_cursor_run/1,
      mng_cursor_materialize/2,
      mng_get_dict/3,
      mng_query_value/2,
      mng_typed_value/2,
      mng_strip/4,
      mng_strip_type/3,
      mng_strip_operator/3,
      mng_strip_variable/2,
      mng_operator/2
    ]).
/** <module> A mongo DB client for Prolog.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).
:- use_module(library('utility/filesystem'),
	[ path_concat/3 ]).

:- use_foreign_library('libmongo_kb.so').

:- dynamic mng_db_name/1.

% define some settings
:- setting(db_name, atom, roslog,
	'Name of the Mongo DB used by KnowRob.').
:- setting(mng_client:collection_prefix, atom, '',
	'ID of the current neem. Empty if neemhub is not used').
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
	(	(setting(mng_client:collection_prefix, Id), Id \= '') 
	->	atomic_list_concat([Id,'_',DBType], CollectionName)
	;	CollectionName = DBType
	).

%% mng_one_db(-DB, -Coll) is det.
%
% Get a special DB collection with just one empty document.
% This is used for feeding just this one document into aggregate
% pipelines.
%
mng_one_db(DB, Coll) :-
	mng_db_name(DB),
	Coll=one.

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
mng_collection(DB, Collection) :-
	mng_collections(DB, Collections),
	memberchk(Collection, Collections).

%%
mng_distinct_values(DB, Collection, Key, DistinctValues) :-
	mng_distinct_values_json(DB, Collection, Key, DistinctJSON),
	read_json_(DistinctJSON, DistinctDict),
	get_dict(values, DistinctDict, ValuesMng),
	findall(V,
		(	member(V_mng,ValuesMng),
			mng_doc_value(V_mng,V)
		),
		DistinctValues
	).

read_json_(JSON,Dict) :-
	atom_to_chars(JSON,Chars),
	open_chars_stream(Chars,Stream),
	json_read_dict(Stream,Dict).

%% mng_find(+DB, +Collection, +Filter, -Result) is nondet.
%
% Creates a cursor with given filter and yields its results.
%
mng_find(DB, Collection, Filter, Result) :-
	setup_call_cleanup(
		% setup: create a query cursor
		(	mng_cursor_create(DB, Collection, Cursor),
			mng_cursor_filter(Cursor, Filter)
		),
		% call: find matching document
		mng_cursor_materialize(Cursor, Result),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
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
mng_regex_prefix(Prefix, Pattern) :-
	atom_codes(Prefix, AtomCodes),
	mng_regex_prefix_(AtomCodes, PatternCodes),
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
mng_cursor_materialize(Cursor,Next) :-
	mng_cursor_next(Cursor,X),
	% pull next result and avoid choicepoint in case
	% there is no additional result available.
	mng_cursor_materialize(Cursor,X,Next).

mng_cursor_materialize(Cursor,First,Next) :-
	mng_cursor_next(Cursor,Second),!,
	(	Next=First
	;	mng_cursor_materialize(Cursor,Second,Next)
	).
mng_cursor_materialize(_,Next,Next).

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
	(	setting(mng_client:read_only, true)
	->	true
    ;	mng_index_create_core(DB,Collection,Keys0)
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
% % % % % typed terms

%%
%
% Creates a document for querying a value of a field
% in mongo. The input value can be a term:
% `$operator(unit($type($value),$unit))->Var`.
%
mng_query_value(Value0, [Operator, Value1]) :-
	% remove _->_ expression for vars 
	mng_strip_variable(Value0, X0),
	% rest term must be grounded
	ground(X0),
	% get the mongo DB operator e.g. $eq
	mng_strip_operator(X0, Operator0, X1),
	mng_operator(Operator0,Operator),
	% ensure the value is wrapped in type term
	mng_strip_type(X1, Type, X2),
	% convert terms to atoms for storage
	(	(Type=term,compound(X2))
	->	term_to_atom(X2,X3)
	;	X3=X2
	),
	% finally wrap value in type again
	type_mapping_(Type, MngType),
	mng_strip_type(Value1, MngType, X3).

%%
mng_typed_value(Term, TypedValue) :-
	mng_strip(Term, _Operator, Type, Value),
	mng_strip_type(TypedValue, Type, Value).

%%
mng_strip(Term, Operator, Type, Value) :-
	mng_strip_variable(Term, Term0),
	mng_strip_operator(Term0, Operator, Term1),
	mng_strip_type(Term1, Type, Value).

%% mng_strip_type(+TypedValue,?Type,?Value) is det.
%
% DocValue is a typed json document value.
% That is, e.g. `TypedValue=int(7)` in which case `Type=int` and `Value=7`.
%
% @TypedValue a value returned by mongo DB
% @Type type atom
% @Value the value without type
%
mng_strip_type(Var, Type, Untyped) :-
	var(Var),
	ground(Type),
	ground(Untyped),!,
	Var =.. [Type, Untyped].

mng_strip_type(Var, _, Var) :-
	var(Var),
	!.

mng_strip_type(List, array, List) :-
	is_list(List),
	!.

mng_strip_type(Term, Type, X) :-
	compound(Term),
	!,
	Term=..[Type,X],
	type_mapping_(Type,_).

mng_strip_type(X, double, X) :-
	number(X),
	!.
% FIXME: below makes it impossible to ask for string values true/false
mng_strip_type(X, bool, X) :-
	ground(X),
	(	X=true
	;	X=false
	),
	!.

% FIXME var(X) always ends in string, but holds takes care of setting type
%                  better do not require type in query!
mng_strip_type(X, string, X).

%%
type_mapping_(float,   double) :- !.
type_mapping_(number,  double) :- !.
type_mapping_(integer, int) :- !.
type_mapping_(long,    int)    :- !.
type_mapping_(short,   int)    :- !.
type_mapping_(byte,    int)    :- !.
type_mapping_(term,    string) :- !.
type_mapping_(X,       X)      :- !.

%%
mng_strip_operator(    X,    =, X) :- var(X), !.
mng_strip_operator( =(X),    =, X) :- !.
mng_strip_operator(>=(X),   >=, X) :- !.
mng_strip_operator(=<(X),   =<, X) :- !.
mng_strip_operator( <(X),    <, X) :- !.
mng_strip_operator( >(X),    >, X) :- !.
mng_strip_operator(in(X),   in, X) :- !.
mng_strip_operator(nin(X), nin, X) :- !.
mng_strip_operator(    X,    =, X) :- !.

%%
mng_operator('=', '$eq').
mng_operator('>=','$gte').
mng_operator('=<','$lte').
mng_operator('>', '$gt').
mng_operator('<', '$lt').
mng_operator('in', '$in').
mng_operator('nin', '$nin').
mng_operator('size', '$size').

%%
mng_strip_variable(X->_,X) :- nonvar(X), !.
mng_strip_variable(X,X) :- !.

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
