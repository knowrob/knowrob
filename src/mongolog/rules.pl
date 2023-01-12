:- module(mongolog_rules,
    []).

:- op(1100, xfx, user:(?>)).
:- op(1100, xfx, user:(+>)).
:- op(1100, xfx, user:(?+>)).

:- use_module(library('semweb/rdf_db'),
        [ rdf_global_term/2 ]).
:- use_module(library('scope')).
:- use_module('mongolog',
        [ mongolog_add_rule/2 ]).

:- dynamic kb_predicate/1.

%%
% Term expansion for *querying* rules using the (?>) operator.
% The body is rewritten such that mng_ask is called instead
% with body as argument.
%
user:term_expansion((?>(Head,Body)), Export) :-
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,Module,Term),
	once((ground(Module);prolog_load_context(module, Module))),
	% add the rule to the DB backend
	mongolog_add_rule(Term, BodyGlobal),
	% expand into regular Prolog rule only once for all clauses
	Term =.. [Functor|Args],
	length(Args,NumArgs),
	length(Args1,NumArgs),
	Term1 =.. [Functor|Args1],
	(	kb_predicate(Term1)
	->	Export=[]
	;	(
		assertz(kb_predicate(Term1)),
		current_scope(QScope),
		Export=[(:-(Term1, mongolog:mongolog_call(Term1, [scope(QScope)])))]
	)).

%%
% Term expansion for *project* rules using the (+>) operator.
% The rules are only asserted into mongo DB and expanded into
% empty list.
%
user:term_expansion((+>(Head,Body)), []) :-
    mongolog:mongolog_consult3((+>(Head,Body)), [load]).
/*
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,_Module,Term),
	% rewrite functor
	% TODO: it would be nicer to generate a lot
	%        clauses for project/1.
	Term =.. [Functor|Args],
	atom_concat('project_',Functor,Functor0),
	Term0 =.. [Functor0|Args],
	% add the rule to the DB backend
	mongolog_add_rule(Term0, project(BodyGlobal)).
*/

%%
% Term expansion for *query+project* rules using the (?+>) operator.
% These are basically clauses that can be used in both contexts.
%
% Consider for example following rule:
%
%     is_event(Entity) ?+>
%       has_type(Entity, dul:'Event').
%
% This is valid because, in this case, has_type/2 has
% clauses for querying and projection.
%
user:term_expansion((?+>(Head,Goal)), X1) :-
	user:term_expansion((?>(Head,Goal)),X1),
	user:term_expansion((+>(Head,Goal)),_X2).

%%
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).
