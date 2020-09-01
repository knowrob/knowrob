:- module(mng_annotations,
    [ annotation_init/0,
      annotation_import/1,
      annotation_export/1,
      annotation_drop/0,
      annotation_tell/3,
      annotation_ask/3,
      annotation_erase/3
    ]).
/** <module> Interface for storing annotations.

This is done separately from storing triples mainly because
the value of triples must be indexed, but the value of an
annotation may be very long which is not ok for indexed fields.
It is also not needed that annotation values are indexed.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('utility/filesystem'),
	[ path_concat/3 ]).
:- use_module('../client.pl').

%%
% The mongo DB + Collection used to store annotations.
%
annotation_db(DB, Name) :- 
	mng_get_db(DB, Name, 'annotations').


%%
% Create indices for fast annotation retrieval.
%
annotation_init :-
	annotation_db(DB,Coll),
	mng_index_create(DB,Coll,['s']),
	mng_index_create(DB,Coll,['p']),
	mng_index_create(DB,Coll,['s','p']).

%%
% Import annotations from a DB dump.
%
annotation_import(Dir) :-
	annotation_db(DB,Coll),
	path_concat(Dir,Coll,Dir0),
	mng_restore(DB,Dir0).

%%
% Export the annotation DB to file.
%
annotation_export(Dir) :-
	annotation_db(_DB,Coll),
	path_concat(Dir,Coll,Dir0),
	mng_export_collection(Coll,Dir0).

%%
% Erases all annotations.
%
annotation_drop :-
	annotation_db(DB,Coll),
	mng_drop(DB,Coll).

%%
% Add an annotation to the DB.
%
annotation_tell(Entity,Property,Annotation) :-
	annotation_ask(Entity,Property,Annotation),
	!.

annotation_tell(Entity,Property,Annotation) :-
	annotation_db(DB,Coll),
	once((	Annotation=string(Stripped)
	;		Annotation=Stripped
	)),
	% TODO: add another field for language?
	once((	Stripped=lang(_,Stripped0)
	;		Stripped=Stripped0
	)),
	% enforce UTF8 encoding
	atom_codes(Stripped0, Codes),
	phrase(utf8_codes(Codes), UTF8),
	% finally write to DB
	mng_store(DB,Coll,[
		['s', string(Entity)],
		['p', string(Property)],
		['v', string(UTF8)]
	]).

%% 
% Get annotations of an entity.
%
annotation_ask(Entity,Property,Annotation) :-
	setup_call_cleanup(
		% setup: create a query cursor
		get_query_cursor_(Entity,Property,Cursor),
		% call: find matching document
		unify_query_cursor_(Cursor,Entity,Property,Annotation),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
% Erase matching annotations.
%
annotation_erase(Subject,Property,_Value) :-
	setup_call_cleanup(
		% setup: create a query cursor
		get_query_cursor_(Subject,Property,Cursor),
		% call: delete all matching documents
		mng_cursor_erase(Cursor),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%% create a query cursor
get_query_cursor_(Subject,Property,Cursor) :-
	annotation_db(DB,Coll),
	%%
	mng_cursor_create(DB,Coll,Cursor),
	( \+ground(Subject)
	; mng_cursor_filter(Cursor,['s',string(Subject)])
	),
	( \+ground(Property)
	; mng_cursor_filter(Cursor,['p',string(Property)])
	),
	!.

%%
unify_query_cursor_(Cursor,Subject,Property,Value) :-
	mng_cursor_materialize(Cursor,Doc),
	( ground(Subject)
	-> true
	;  mng_get_dict('s',Doc,string(Subject))
	),
	( ground(Property)
	-> true
	;  mng_get_dict('p',Doc,string(Property))
	),
	mng_get_dict('v',Doc,string(Value)).
