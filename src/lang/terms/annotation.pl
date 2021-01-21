:- module(lang_annotation, []).

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1
	    ]).

:- use_module(library('db/mongo/compiler')).
:- use_module(library('db/mongo/query')).

:- rdf_meta(query_(t,t,t,t,-)).

% TODO: language handling.
%	- e.g. DUL has comments in different langs
%

%%
% register the "annotations" collection.
% This is needed for import/export.
%
:- kb_collection_init(annotations).

%%
% Create indices for fast annotation retrieval.
%
create_search_inidces_ :-
	setting(mng_client:read_only, true),
	!.
create_search_inidces_ :-
	mng_get_db(DB, Coll, 'annotations'),
	mng_index_create(DB, Coll, ['s']),
	mng_index_create(DB, Coll, ['p']),
	mng_index_create(DB, Coll, ['s','p']).

:- create_search_inidces_.

%% register query commands
:- add_query_command(comment).
%:- add_query_command(seeAlso).

%%
% expose argument variables.
%
query_compiler:step_var(
		comment(Entity, Comment),
		[Key, Var]) :-
	(	annotation_var_(Entity, [Key, Var])
	;	annotation_var_(Comment, [Key, Var])
	).

%%
annotation_var_(Arg, [Key, Var]) :-
	mng_strip_type(Arg,_,Var),
	query_compiler:var_key(Var, Key).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% ASK
%%%%%%%%%%%%%%%%%%%%%%%

%%
% ask(comment(Entity, Comment)) looks up the comment
% of entities. These are stored in a separate collection
% to avoid generating a regular index over the comment values. 
%
query_compiler:step_compile(comment(S, C), Ctx, Pipeline) :-
	option(ask, Ctx), !,
	query_(S, rdfs:comment, C, Ctx, Pipeline).

%% 
query_(Entity, Property, Comment, Context, Pipeline) :-
	% get the DB collection
	mng_get_db(_DB, Coll, 'annotations'),
	% extend the context
	Context0 = [collection(Coll)|Context],
	% compute steps of the aggregate pipeline
	findall(Step,
		% look-up comments into 'next' field
		(	lookup_(Entity, Property,
				Comment, Context0, Step)
		;	Step=['$unwind',string('$next')]
		% set variable field
		;	set_result_(Context0, Step)
		% remove next field
		;	Step=['$unset',string('next')]
		),
		Pipeline
	).

%% 
lookup_(Entity, Property, Comment, Context, Step) :-
	% get filter for Entity/Property
	findall(X,
		(	( ground(Entity),   X=['s',string(Entity)] )
		;	( ground(Property), X=['p',string(Property)] )
		),
		QueryDoc),
	% we can use generator for triple command here
	lang_triple:lookup_1(
		triple(Entity, Property, Comment),
		QueryDoc, Context, Step).

%%
set_result_(Comment,
		['$set', [Key, string('$next.v')]]) :-
	mng_strip_type(Comment,_,Var),
	query_compiler:var_key(Var, Key).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% TELL
%%%%%%%%%%%%%%%%%%%%%%%

%%
% TODO: support tell(comment)
% tell(comment(Entity, Comment)) adds a comment to an entity.
% comments may contain special characters.
%
%mng_compiler:step_compile(
%		comment(Entity, Comment),
%		Context,
%		Pipeline) :-
%	option(tell, Context), !,
%	 	- but not possible with $merge into triples collection!!
%	
%	annotation_db(DB,Coll),
%	once((	Annotation=string(Stripped)
%	;		Annotation=Stripped
%	)),
%	% TODO: add another field for language?
%	once((	Stripped=lang(_,Stripped0)
%	;		Stripped=Stripped0
%	)),
%	% enforce UTF8 encoding
%	atom_codes(Stripped0, Codes),
%	phrase(utf8_codes(Codes), UTF8),
%	% finally write to DB
%	mng_store(DB,Coll,[
%		['s', string(Entity)],
%		['p', string(Property)],
%		['v', string(UTF8)]
%	]).
