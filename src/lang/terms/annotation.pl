:- module(lang_annotation, []).
/** <module> Handling of annotations in query expressions.

The following predicates are supported:

| Predicate            | Arguments |
| ---                  | ---       |
| annotation/3         | ?Subject, ?Property, ?Value |

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1 ]).
:- use_module(library('db/mongo/client'),
		[ mng_get_db/3, mng_strip_type/3 ]).
:- use_module(library('lang/db')).
:- use_module(library('lang/mongolog/mongolog')).

:- rdf_meta(query_annotation(+,r,+,+,-)).

%%
% register the "annotations" collection.
% This is needed for import/export and search indices.
%
:- setup_collection(annotations,
		[['s'], ['p'], ['s','p']]).

%% query commands
:- mongolog:add_command(annotation).

%%
lang_query:step_expand(
	project(annotation(S,P,O)),
	assert(annotation(S,P,O))) :- !.

%% annotation(+Entity, +Property, -Annotation)
%
mongolog:step_compile(
		assert(annotation(Entity, Property, Annotation)),
		Ctx, Pipeline, StepVars) :-
	assert_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars).

mongolog:step_compile(
		annotation(Entity, Property, Annotation),
		Ctx, Pipeline, StepVars) :-
	query_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars).

%% 
query_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars) :-
	% throw instantiation_error if one of the arguments was not referred to before
	mongolog:all_ground([Entity, Property], Ctx),
	mongolog:step_vars([Entity,Property,Annotation], Ctx, StepVars),
	% get the DB collection
	mng_get_db(_DB, Coll, 'annotations'),
	mongolog:var_key_or_val(Annotation,  Ctx, Annotation0),
	mongolog:var_key_or_val1(Entity,     Ctx, Entity0),
	mongolog:var_key_or_val1(Property,   Ctx, Property0),
	% pass input document values to lookup
	mongolog:lookup_let_doc(StepVars, LetDoc),
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
		;	mongolog:set_if_var(Annotation, string('$next.v'), Ctx, Step)
		;	mongolog:match_equals(Annotation0, string('$next.v'), Step)
		;	Step=['$unset',string('next')]
		),
		Pipeline
	).

%%
assert_annotation(Entity, Property, Annotation, Ctx, Pipeline, StepVars) :-
	mng_strip_type(Annotation, _, UnTyped),
	% only load annotations written in English
	% silently do nothing for other languages
	(	strip_lang(UnTyped, en, Stripped)
	->	assert_annotation(Entity, Property, Annotation,
			Stripped, Ctx, Pipeline, StepVars)
	;	(Pipeline=[], StepVars=[])
	).

assert_annotation(Entity, Property, Annotation, Stripped, Ctx, [Step], StepVars) :-
	mng_get_db(_DB, Collection, 'annotations'),
	% enforce UTF8 encoding
	utf8_value(Stripped, Annotation_en),
	% throw instantiation_error if one of the arguments was not referred to before
	mongolog:all_ground([Entity, Property, Annotation_en], Ctx),
	mongolog:step_vars([Entity,Property,Annotation], Ctx, StepVars0),
	mongolog:add_assertion_var(StepVars0, StepVars),
	% resolve arguments
	mongolog:var_key_or_val(Entity,         Ctx, Entity0),
	mongolog:var_key_or_val(Property,       Ctx, Property0),
	mongolog:var_key_or_val(Annotation_en,  Ctx, Annotation0),
	% get the query
	mongolog:add_assertion([
				['s', Entity0],
				['p', Property0],
				['v', Annotation0]
			], Collection, Step).

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

test('project(assign(C,c), annotation(+,+,+))') :-
	assert_true(kb_project((
		ask(assign(C,g)),
		annotation(e,f,C)
	))).

test('project(annotation(+,+,+))') :-
	assert_true(kb_project(annotation(a,b,c))).

test('project(annotation(+,+,-))', [throws(error(instantiation_error,project(annotation(a,b,_))))]) :-
	kb_project(annotation(a,b,_)).

test('annotation(+,+,-)') :-
	assert_true(kb_call(annotation(a,b,C))),
	(	kb_call(annotation(a,b,C))
	->	assert_equals(C,c)
	;	true
	).

test('annotation(+,+,+)') :-
	assert_true(kb_call(annotation(a,b,c))),
	assert_false(kb_call(annotation(a,b,d))).

test('annotation(-,+,+)', [throws(error(instantiation_error,_))]) :-
	kb_call(annotation(_,b,c)).

test('annotation(+,-,+)', [throws(error(instantiation_error,_))]) :-
	kb_call(annotation(a,_,c)).

test('annotation(-,+,-)', [throws(error(instantiation_error,_))]) :-
	kb_call(annotation(_,b,_)).

:- end_tests('lang_annotation').
