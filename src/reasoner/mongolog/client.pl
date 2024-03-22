:- module(mng_client,
    [ mng_collection/2,
      mng_distinct_values/4,
      mng_drop/2,
      mng_store/3,
      mng_update/4,
      mng_remove/3,
      mng_find/4,
      mng_index_create/2,
      mng_index_create/3,
      mng_dump/2,
      mng_dump/3,
      mng_dump_collection/3,
      mng_restore/2,
      mng_restore/3,
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
      mng_query_value/2,
      mng_typed_value/2,
      mng_unflatten/2,
      mng_strip/4,
      mng_strip_type/3,
      mng_strip_operator/3,
      mng_strip_variable/2,
      mng_operator/2
    ]).
/** <module> A mongo DB client for Prolog.

This module provides access to a large part of the
mongo DB API includig several read and write operations.
Internally, libmongoc is used to interact with the database server.
To this end, Prolog datastructures are translated from and into BSON format.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('logging')).
:- use_module(library('http/json')).
:- use_module(library('ext/filesystem'), [ path_concat/3 ]).

%:- use_foreign_library('libmongo_kb.so').

%% mng_drop(+DB,+Collection) is det.
%
% Drop a named collection. That is delete all documents
% it contains, and remove all references to it in the database.
%
% @param DB The database name
% @param Collection The collection name
% @see https://docs.mongodb.com/manual/reference/method/db.collection.drop/index.html
%
mng_drop(DB,Collection) :-
	catch(mng_drop_unsafe(DB,Collection),
		  mng_error(drop_failed(Collection)),
		  true).

%% mng_collection(+DB,?Collection) is nondet
%
% True if Collection is an existing collection
% in the named database.
%
% @param DB The database name
% @param Collection The collection name
%
mng_collection(DB, Collection) :-
	mng_collections(DB, Collections),
	memberchk(Collection, Collections).

%% mng_distinct_values(+DB, +Collection, +Key, -DistinctValues) is det
%
% Find all distinct values associated to Key in
% the given named database collection.
%
% @param DB The database name
% @param Collection The collection name
% @param Key The document key of interest
% @param DistinctValues List of distinct values
%
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
	open_chars_stream(Chars,QueryStage),
	json_read_dict(QueryStage,Dict).

%% mng_find(+DB, +Collection, +Filter, -Result) is nondet.
%
% Create a database cursor with given filter query and yield its results.
% The filter must be translatable into a BSON document, and be given as
% list datastructure as in:
%
%    mng_find(roslog, triples, ['s',['$eq',string('Obj1')]], Result)
%
% Result is a Prolog dictionary instantiated from the JSON document
% returned by mongo DB.
%
% @param DB The database name
% @param Collection The collection name
% @param Filter A mongo DB query
% @param Result A document matching the query
% @see https://docs.mongodb.com/manual/reference/method/db.collection.find/index.html
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
% Yields the next matching document of the given
% database cursor, if any.
% The matching document is encoded as Prolog dictionary.
%
% @param Cursor The database cursor.
% @param Dict The next matching document of the cursor.
% @see http://mongoc.org/libmongoc/current/mongoc_cursor_next.html
%
mng_cursor_next(Cursor,Dict) :-
	mng_cursor_next_pairs(Cursor,Pairs),
	dict_pairs(Dict,_,Pairs).

%% mng_cursor_materialize(+Cursor,?Dict) is nondet.
%
% Yields results of the given database cursor, if any.
% Each matching document is encoded as Prolog dictionary.
%
% @param Cursor The database cursor.
% @param Dict The next matching document of the cursor.
% @see http://mongoc.org/libmongoc/current/mongoc_cursor_next.html
%
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
% Creates a compound search index.
%
% @param DB The database name
% @param Collection The collection name
% @param Keys List of keys for which an index shall be created
% @see https://docs.mongodb.com/manual/core/index-compound/
%
mng_index_create(DB,Collection,Keys) :-
	findall(K,
		(	member(K0,Keys),
			format_key_(K0,K)
		),
		Keys0
	),
	(	reasoner_setting(mongodb:read_only, true)
	->	true
    ;	mng_index_create_core(DB,Collection,Keys0)
    ).

%% mng_index_create(+DB,+Indices) is det
%
% Creates compound search indices.
% Indices is a list of tuples where the first
% argument is the name of the collection, and
% the second argument a list of keys passed
% to mng_index_create/3.
%
% @param DB The database name
% @param Indices Sequence of search indices
% @see https://docs.mongodb.com/manual/core/index-compound/
%
mng_index_create(DB,Indices) :-
	forall(member([Coll,Keys],Indices),
		   mng_index_create(DB,Coll,Keys)).

%%
format_key_(+(K),+(K)) :- !.
format_key_(-(K),-(K)) :- !.
format_key_(  K, +(K)) :- !.


%% mng_get_dict(?Key,+Doc,?PlValue) is semidet.
%
% Get a key-value pair from a dictionary.
% If the value is a document, it will be
% mapped to a Prolog dictionary.
% This is done recursively.
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

%% mng_dump(+DB, +Directory) is det.
%
% Dump a named database by calling the `mongodump` commandline
% tool.
%
% @param DB the database name
% @param Directory absolute path to output directory
%
mng_dump(DB,Dir) :-
	mng_dump(DB,Dir,_).

mng_dump(DB,Dir,Output) :-
	mongolog_database:mongolog_uri(URI),
	process_create(path(mongodump),
		[ '--uri', URI, '--db', DB, '--out', Dir ],
		[ process(PID), stderr(pipe(StdErrStream)) ]
	),
	read_lines(StdErrStream, Output),
	wait(PID,exited(0)).

%% mng_dump_collection(+DB, +Collection, +Directory) is det.
%
% Dump a named database collection by calling the `mongodump` commandline
% tool.
%
% @param DB the database name
% @param Collection The collection name
% @param Directory absolute path to output directory
%
mng_dump_collection(DB,Collection,Directory) :-
	process_create(path(mongodump),
		[ '--db', DB, '--collection', Collection, '--out', Directory ],
		[ process(PID) ]
	),
	wait(PID,exited(0)).

%% mng_restore(+DB, +Directory) is det.
%
% Restore named database by calling the `mongorestore` commandline
% tool.
%
% @param DB the database name
% @param Directory absolute path to output directory
%
mng_restore(DB,Dir) :-
	mng_restore(DB,Dir,_).

mng_restore(DB,Dir,Output) :-
	mongolog_database:mongolog_uri(URI),
	process_create(path(mongorestore),
		[ '--uri', URI, '--db', DB, '--dir', Dir, '--quiet' ],
		[ process(PID), stderr(pipe(StdErrStream)) ]
	),
	read_lines(StdErrStream, Output),
	wait(PID,exited(0)).	


% copied from https://www.swi-prolog.org/pldoc/man?predicate=process_create/3
%
% Read lines of Out-Stream, usually for reading stdout and stderr as
% list of lines
%
% @param Out the stream, created by pipe(Out)
% @param Lines list of lines that were read
%
read_lines(Out, Lines) :-
        read_line_to_codes(Out, Line1),
        read_lines(Line1, Out, Lines).

read_lines(end_of_file, _, []) :- !.
read_lines(Codes, Out, [Line|Lines]) :-
        atom_codes(Line, Codes),
        read_line_to_codes(Out, Line2),
        read_lines(Line2, Out, Lines).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % typed terms

%% mng_query_value(+Term, -Document) is semidet.
%
% Creates a query document from a query term.
% The input value can optionally be wrapped in
% a unary type term as in `double(4)`.
% It can further be wrapped in a unary operator term as
% in `<(4)`.
% In such a case, it might be useful to retrieve the actual value too.
% This can be achieved through ->/2 operator as in  `<(4)->Actual`.
%
% @param Term The query term
% @param Document the query document
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
	(	Type==term
	->	term_document(X2, Value1)
	% finally wrap value in type again
	;	(	type_mapping(Type, MngType),
			mng_strip_type(Value1, MngType, X2)
		)
	).
	% convert terms to atoms for storage
%	(	(Type=term,compound(X2))
%	->	term_to_atom(X2,X3)
%	;	X3=X2
%	),
%	type_mapping(Type, MngType),
%	mng_strip_type(Value1, MngType, X3).

%%
term_document(Term, [
		['type', string('compound')],
		['value', [
			['functor', string(Functor)],
			['args', array(ArgDocs)]
		]]
	]) :-
	Term =.. [Functor|Args],
	maplist([Arg,Doc]>>
		mng_query_value(Arg, ['$eq', Doc]),
		Args, ArgDocs).

%% mng_unflatten(+Flat, -Nested) is det.
%
% Translates a flattened document into a nested one.
% Flattened documents may contain nested keys that contain
% a '.'. These are translated into a nested structure instead.
% For example: `mng_unflatten(['a.b',1], [a,[b,1]])`.
%
%
mng_unflatten(Flat, Nested) :-
	mng_unflatten(Flat, [], Nested).

mng_unflatten([], Nested, Nested) :- !.
mng_unflatten([X|Xs], NestedIn, NestedOut) :-
	unflatten_entry(X, NestedX),
	mng_unflatten1(NestedX, NestedIn, Nested0),
	mng_unflatten(Xs, Nested0, NestedOut).

mng_unflatten1([[Key,Rest]], NestedIn,
		[[Key,NestedKey]|NestedRest]) :-
	is_list(Rest),
	select([Key,Ys0], NestedIn, NestedRest),
	mng_unflatten1(Rest, Ys0, NestedKey),
	!.
mng_unflatten1(X, NestedIn, NestedOut) :-
	append(X, NestedIn, NestedOut).

%%
unflatten_entry([Path,Value], Nested) :-
	atomic_list_concat(Keys, '.', Path),
	unflatten_entry(Keys, Value, Nested).
unflatten_entry([], Value, Value) :- !.
unflatten_entry([X|Xs], Value, [[X,Rest]]) :-
	unflatten_entry(Xs, Value, Rest).

%% mng_typed_value(+Term, -TypedValue) is det.
%
% Ensure that Term includes a unary type term.
% For example:
%
%    mng_typed_value(foo, string(foo))
%
% @param Term A potentially untyped value term.
% @param TypedValue A typed value term.
%
mng_typed_value(Term, TypedValue) :-
	mng_strip(Term, _Operator, Type, Value),
	mng_strip_type(TypedValue, Type, Value).

%% mng_operator(?PlOperator, ?MngOperator) is det.
%
% A mapping between Prolog operators and mongo DB operators.
%
% @param PlOperator A Prolog operator such as '<'
% @param MngOperator A mongo operator such as '$lt'
%
mng_operator('=', '$eq').
mng_operator('>=','$gte').
mng_operator('=<','$lte').
mng_operator('>', '$gt').
mng_operator('<', '$lt').
mng_operator('in', '$in').
mng_operator('nin', '$nin').

%% mng_strip(+Term, ?Operator, ?Type, ?Value) is semidet.
%
% Strips a value term from its operator and type.
% For example:
%
%    mng_strip(<(double(2)), <, double, 2)
%
%    mng_strip(2, =, double, 2)
%
%    mng_strip(<(2)->X, <, double, 2)
%
% @param Term A value term.
% @param Operator The stripped operator or '='.
% @param Type The stripped type or default type for value.
% @param Value The bare value.
%
mng_strip(Term, Operator, Type, Value) :-
	mng_strip_variable(Term, Term0),
	mng_strip_operator(Term0, Operator, Term1),
	mng_strip_type(Term1, Type, Value).

%% mng_strip_type(+Term, ?Type, ?Value) is det.
%
% Strip the type of a value term.
% That is, e.g. `Term=int(7)` in which case `Type=int` and `Value=7`.
% If Term is untyped, the type will be determined through the
% Prolog datatype of the value.
%
% @param Term a value term
% @param Type type atom
% @param Value the value without type
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
	Term=..[Type,X],
	type_mapping(Type,_),
	!.

mng_strip_type(Term, term, Term) :-
	compound(Term),
	!.

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

% FIXME var(X) always ends in string, better do not require type in query!
mng_strip_type(X, string, X).

%%
type_mapping(double,  double) :- !.
type_mapping(float,   double) :- !.
type_mapping(number,  double) :- !.
type_mapping(int,     int)    :- !.
type_mapping(integer, int)    :- !.
type_mapping(long,    int)    :- !.
type_mapping(short,   int)    :- !.
type_mapping(byte,    int)    :- !.
type_mapping(string,  string) :- !.
type_mapping(atom,    string) :- !.
type_mapping(regex,   regex) :- !.
type_mapping(array,   array)  :- !.
type_mapping(list,    array)  :- !.
type_mapping(bool,    bool)   :- !.
type_mapping(time,    time)   :- !.
%type_mapping(term,    string) :- !.

%% mng_strip_operator(+Term, ?Operator, ?Stripped) is det.
%
% Strip the operator of a value term.
% That is, e.g. `Term=(<(7))` in which case `Operator='<'` and `Stripped=7`.
% If Term has no operator, then equality operator is used as fallabck.
%
% @param Term A value term
% @param Operator The stripped operator or '='
% @param Stripped The value term without operator
%
mng_strip_operator(    X,    =, X) :- var(X), !.
mng_strip_operator( =(X),    =, X) :- !.
mng_strip_operator(>=(X),   >=, X) :- !.
mng_strip_operator(=<(X),   =<, X) :- !.
mng_strip_operator( <(X),    <, X) :- !.
mng_strip_operator( >(X),    >, X) :- !.
mng_strip_operator(in(X),   in, X) :- !.
mng_strip_operator(nin(X), nin, X) :- !.
mng_strip_operator(    X,    =, X) :- !.

%% mng_strip_variable(+Term, ?Stripped) is det.
%
% Strips variable from a value term.
% That is, e.g. `Term=(<(7)->X)` in which case `Stripped=(<(7))`.
%
% @param Term A value term
% @param Stripped The value term without variable
%
mng_strip_variable(X->_,X) :- nonvar(X), !.
mng_strip_variable(X,X) :- !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % C++ predicates

%% mng_store(+DB, +Collection, +Document)
%
% Stores a document in a named database collection.
% Document must be translatable into a BSON document,
% e.g. `Document=[[foo,string(bar)]]` would create a document
% with string value "bar" assigned to field with key "foo".
%
% @param DB The database name
% @param Collection The collection name
% @param Document A database document
%

%% mng_remove(+DB, +Collection, +Query)
%
% Removes all documents matching Query from a named collection.
% Query must be translatable into a BSON document,
% e.g. `Query=[key,['$lt',double(2)]]`.
%
% @param DB The database name
% @param Collection The collection name
% @param Query A query document
% @see https://docs.mongodb.com/manual/reference/method/db.collection.remove/index.html
%

%% mng_update(+DB, +Collection, +Query, +Update)
%
% Updates all documents in a named collection that match Query.
% Query und Update must be translatable into a BSON document,
% e.g. `Query=[key,['$lt',double(2)]]`.
%
% @param DB The database name
% @param Collection The collection name
% @param Query A query document
% @param Update A update document or pipeline
% @see https://docs.mongodb.com/manual/reference/method/db.collection.update/index.html
%

%% mng_cursor_create(+DB, +Collection, -Cursor) is det.
%
% Creates a new query cursor.
% Make sure to call mng_cursor_destroy/1 once
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
% Limit the maximum number of documents a cursor may yield.
%
% @param Cursor A mongo DB cursor id
% @param Limit The maximum number of documents yielded by the cursor
%

%% mng_cursor_descending(+Cursor, +Key) is det.
%
% Configure a cursor to yield documents in descending order.
%
% @param Cursor A mongo DB cursor id
% @param Key The sort key
%

%% mng_cursor_ascending(+Cursor, +Key) is det.
%
% Configure a cursor to yield documents in ascending order.
%
% @param Cursor A mongo DB cursor id
% @param Key The sort key
%

%% mng_cursor_filter(+Cursor, +Query) is det.
%
% Appends an additional condition for documents matching the cursor.
% Query is a query term that must be translatable into a BSON document.
%
% @param Cursor A mongo DB cursor id
% @param Query A query document
%
