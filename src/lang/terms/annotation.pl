:- module(lang_annotation, []).

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1 ]).
:- use_module(library('db/mongo/client'),
		[ mng_get_db/3, mng_strip_type/3 ]).
:- use_module(library('lang/db')).
:- use_module(library('lang/compiler')).

:- rdf_meta(query_annotation(+,r,+,+,-)).

%%
% register the "annotations" collection.
% This is needed for import/export and search indices.
%
:- setup_collection(annotations,
		[['s'], ['p'], ['s','p']]).

%% query commands
:- query_compiler:add_command(annotation).

%% annotation(+Entity, +Property, -Annotation)
%
query_compiler:step_compile(
		annotation(Entity, Property, Annotation),
		Ctx, Pipeline, StepVars) :-
	query_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars).

%% 
query_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars) :-
	option(mode(ask),Ctx),!,
	% throw instantiation_error if one of the arguments was not referred to before
	query_compiler:all_ground([Entity, Property], Ctx),
	query_compiler:step_vars([Entity,Property,Annotation], Ctx, StepVars),
	% get the DB collection
	mng_get_db(_DB, Coll, 'annotations'),
	query_compiler:var_key_or_val(Annotation,  Ctx, Annotation0),
	query_compiler:var_key_or_val1(Entity,     Ctx, Entity0),
	query_compiler:var_key_or_val1(Property,   Ctx, Property0),
	% pass input document values to lookup
	query_compiler:lookup_let_doc(StepVars, LetDoc),
	% compute steps of the aggregate pipeline
	findall(Step,
		% look-up comments into 'next' field
		(	Step=['$lookup', [
				['from', string(Coll)],
				['as', string('next')],
				['let', LetDoc],
				['pipeline', array([['$match', [
					[s, Entity0],
					[p, Property0]
				]]])]
			]]
		% unwind lookup results and assign variable
		;	Step=['$unwind',string('$next')]
		;	query_compiler:set_if_var(Annotation, string('$next.v'), Ctx, Step)
		;	query_compiler:match_equals(Annotation0, string('$next.v'), Step)
		;	Step=['$unset',string('next')]
		),
		Pipeline
	).

query_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars) :-
	option(mode(tell),Ctx),!,
	mng_strip_type(Annotation, _, UnTyped),
	% only load annotations written in English
	% silently do nothing for other languages
	(	strip_lang(UnTyped, en, Stripped)
	->	tell_annotation(Entity, Property, Annotation,
			Stripped, Ctx, Pipeline, StepVars)
	;	(Pipeline=[], StepVars=[])
	).

%%
tell_annotation(Entity, Property, Annotation, Stripped, Ctx,
		[['$set', ['annotations', ['$setUnion',
			array([string('$annotations'),array([[
				['s', Entity0],
				['p', Property0],
				['v', Annotation0]
			]])])
		]]]], StepVars) :-
	% enforce UTF8 encoding
	utf8_value(Stripped, Annotation_en),
	% throw instantiation_error if one of the arguments was not referred to before
	query_compiler:all_ground([Entity, Property, Annotation_en], Ctx),
	query_compiler:step_vars([Entity,Property,Annotation], Ctx, StepVars),
	% resolve arguments
	query_compiler:var_key_or_val(Entity,         Ctx, Entity0),
	query_compiler:var_key_or_val(Property,       Ctx, Property0),
	query_compiler:var_key_or_val(Annotation_en,  Ctx, Annotation0).

%%
utf8_value(Atom, string(UTF8)) :-
	atom(Atom),!,
	atom_codes(Atom, Codes),
	phrase(utf8_codes(Codes), UTF8).
utf8_value(X, string(X)).

%%
strip_lang(Var, en, Var) :- var(Var), !.
strip_lang(lang(Lang0,Val), Lang1, Val) :- !, Lang0 = Lang1.
strip_lang(Val, en, Val).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

test_cleanup :-
	mng_get_db(DB, Coll, 'annotations'),
	mng_remove(DB, Coll, [[s,string(e)]]),
	mng_remove(DB, Coll, [[s,string(a)]]).

:- begin_tests('lang_annotation',
		[ cleanup(lang_annotation:test_cleanup) ]).

test('tell(set(C,c), annotation(+,+,+))') :-
	assert_true(lang_query:tell((
		set(C,g),
		annotation(e,f,C)
	))).

test('tell(annotation(+,+,+))') :-
	assert_true(lang_query:tell(annotation(a,b,c))).

test('tell(annotation(+,+,-))', [throws(error(instantiation_error,annotation(a,b,_)))]) :-
	lang_query:tell(annotation(a,b,_)).

test('annotation(+,+,-)') :-
	lang_query:ask(annotation(a,b,C)),
	assert_equals(C,c).

test('annotation(+,+,+)') :-
	assert_true(lang_query:ask(annotation(a,b,c))),
	assert_false(lang_query:ask(annotation(a,b,d))).

test('annotation(-,+,+)', [throws(error(instantiation_error,annotation(_,b,c)))]) :-
	lang_query:ask(annotation(_,b,c)).

test('annotation(+,-,+)', [throws(error(instantiation_error,annotation(a,_,c)))]) :-
	lang_query:ask(annotation(a,_,c)).

test('annotation(-,+,-)', [throws(error(instantiation_error,annotation(_,b,_)))]) :-
	lang_query:ask(annotation(_,b,_)).

:- end_tests('lang_annotation').
