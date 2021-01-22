:- module(lang_annotation, []).

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1
	    ]).

:- use_module(library('lang/compiler')).

:- rdf_meta(ask_annotation(t,t,t,t,-)).

% TODO: language handling.
%	- e.g. DUL has comments in different langs
%

%%
% register the "annotations" collection.
% This is needed for import/export and search indices.
%
:- setup_collection(annotations,
		[['s'], ['p'], ['s','p']]).

%% register query command
:- query_compiler:add_command(comment).

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

%%
% ask(comment(Entity, Comment)) looks up the comment
% of entities. These are stored in a separate collection
% to avoid generating a regular index over the comment values. 
%
query_compiler:step_compile(comment(S, C), Ctx, Pipeline) :-
	% tell+comment not supported yet
	(	option(mode(tell), Ctx)
	->	throw(compilation_failed(comment(S, C)), Ctx)
	;	true
	),
	%%
	%option(mode(ask), Ctx),
	ask_annotation(S, rdfs:comment, C, Ctx, Pipeline).

%% 
ask_annotation(Entity, Property, Comment, Context, Pipeline) :-
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
% TODO: support tell(comment)
% tell(comment(Entity, Comment)) adds a comment to an entity.
% comments may contain special characters.
%
%tell_annotation(Entity, Property, Comment, Context, Pipeline) :-
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
