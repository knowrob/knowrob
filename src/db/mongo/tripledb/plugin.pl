:- module(mng_tripledb,
    [ implements('db/itripledb')
    ]).

:- use_module('./triples.pl').
:- use_module('./annotations.pl').
:- use_module('./inference_cache.pl').

% define some settings
:- setting(annotation_properties, list,
	[ 'http://www.w3.org/2000/01/rdf-schema#comment',
	  'http://www.w3.org/2000/01/rdf-schema#seeAlso',
	  'http://www.w3.org/2002/07/owl#versionInfo'
	],
	'List of annotation properties.').

:- dynamic is_annotation_property_/1.

%%
% True for properties appearing in the *annotation_properties* setting.
%
is_annotation_property(P) :-
	ground(P),
	is_annotation_property_(P).

%% 
% @implements 'db/itripledb'
%
tripledb_init :-
	%%
	setting(annotation_properties,List),
	retractall(is_annotation_property_(_)),
	forall(
		member(P,List),
		assertz(is_annotation_property_(P))
	),
	%%
	triple_init,
	annotation_init,
	inference_cache_init.

%% 
% @implements 'db/itripledb'
%
tripledb_import(Dir) :-
	triple_import(Dir),
	annotation_import(Dir),
	inference_cache_import(Dir).

%% 
% @implements 'db/itripledb'
%
tripledb_export(Dir) :-
	triples_export(Dir),
	annotations_export(Dir),
	inference_cache_export(Dir).

%% 
% @implements 'db/itripledb'
%
tripledb_drop :-
	triples_drop,
	annotations_drop,
	inference_cache_drop.

%% 
% @implements 'db/itripledb'
%
tripledb_tell(S,P,ValueQuery,Scope,Options) :-
	( is_annotation_property(P)
	-> annotation_tell(S,P,ValueQuery)
	;  triple_tell(S,P,ValueQuery,Scope,Options)
	).

%%
% @implements 'db/itripledb'
%
tripledb_bulk_tell(Triples,Scope,Options) :-
	% TODO: support bulk insert.
	%         - needs support for bulk operations (mongoc_collection_create_bulk_operation)
	%         - make sure no duplicates are added
	%         - still need to propagate subclass/subproperty assertions!
	forall(
		member(rdf(S,P,O),Triples),
		tripledb_tell(S,P,O,Scope,Options)
	).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(S,P,ValueQuery,QScope,FScope,Options) :-
	( is_annotation_property(P)
	-> annotation_ask(S,P,ValueQuery)
	;  triple_ask(S,P,ValueQuery,QScope,FScope,Options)
	).

%%
% @implements 'db/itripledb'
%
tripledb_forget(S,P,ValueQuery,QScope,Options) :-
	( is_annotation_property(P)
	-> annotation_erase(S,P,ValueQuery)
	; (
		triple_erase(S,P,ValueQuery,QScope,Options),
		%% TODO: only invalidate if necessary
		tripledb_cache_invalidate(subclass_of),
		tripledb_cache_invalidate(subproperty_of)
	)).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_get(Predicate,Query,Modules) :-
	inference_cache_get(Predicate,Query,Modules).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_add(Predicate,Query,Module) :-
	inference_cache_add(Predicate,Query,Module).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_invalidate(Predicate) :-
	inference_cache_invalidate(Predicate).
