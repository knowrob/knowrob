:- module(mng_inference_cache,
    [ inference_cache_init/0,
      inference_cache_import/1,
      inference_cache_export/1,
      inference_cache_drop/0,
      inference_cache_get/3,
      inference_cache_add/3,
      inference_cache_invalidate/1
    ]).

:- use_module(library('utility/filesystem'),
	[ path_concat/3 ]).
:- use_module('../client.pl').

:- dynamic inference_cache_empty/1.

%%
% The mongo DB + Collection used to store the query cache.
%
inference_cache_db(DB,'inferred') :-
	mng_db_name(DB).

%%
% Create search indices.
%
inference_cache_init :-
	inference_cache_db(DB,Coll),
	mng_index_create(DB,Coll,['query']),
	mng_index_create(DB,Coll,['predicate']).

%%
% Import from a DB dump.
%
inference_cache_import(Dir) :-
	inference_cache_db(DB,Coll),
	path_concat(Dir,Coll,Dir0),
	mng_restore(DB,Dir0).

%%
% Export to file.
%
inference_cache_export(Dir) :-
	inference_cache_db(_DB,Coll),
	path_concat(Dir,Coll,Dir0),
	mng_export_collection(Coll,Dir0).

%%
% Erases all cached queries.
%
inference_cache_drop :-
	inference_cache_db(DB,Coll),
	mng_drop(DB,Coll).

%%
% Get list of reasoning modules that have cached results
% for some query.
%
inference_cache_get(Predicate,_,[]) :-
	inference_cache_empty(Predicate),!.

inference_cache_get(_Predicate,Query,Modules) :-
	inference_cache_db(DB,Coll),
	mng_cursor_create(DB,Coll,Cursor),
	mng_cursor_filter(Cursor,['query',string(Query)]),
	findall(M,
		(	mng_cursor_materialize(Cursor,Doc),
			mng_get_dict('module',Doc,string(M))
		),
		Modules
	),
	mng_cursor_destroy(Cursor).

%%
% Add a reasoning module to the list of modules
% that have answered some query.
%
inference_cache_add(Predicate,Query,Module) :-
	inference_cache_db(DB,Coll),
	mng_store(DB,Coll,[
		[query,     string(Query)],
		[module,    string(Module)],
		[predicate, string(Predicate)]
	]),
	retractall(inference_cache_empty(Predicate)).

%%
% Erase cached queries for some language predicate.
%
inference_cache_invalidate(Predicate) :-
	inference_cache_empty(Predicate),
	!.

inference_cache_invalidate(Predicate) :-
	inference_cache_db(DB,Coll),
	mng_cursor_create(DB,Coll,Cursor),
	mng_cursor_filter(Cursor,['predicate',string(Predicate)]),
	mng_cursor_erase(Cursor),
	mng_cursor_destroy(Cursor),
	assertz(inference_cache_empty(Predicate)).
