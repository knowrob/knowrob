:- module(mongolog_typecheck, []).
/** <module> Verify type of a term in mongolog programs.

The following predicates are supported:

| Predicate    | Arguments |
| ---          | ---       |
| ground/1     | @Term |
| var/1        | @Term |
| number/1     | @Term |
| atom/1       | @Term |
| is_list/1    | @Term |
| compound/1   | @Term |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=typetest
@license BSD
*/

:- use_module('mongolog').

%% register query commands
:- mongolog:add_command(ground).
:- mongolog:add_command(var).
:- mongolog:add_command(number).
:- mongolog:add_command(atom).
:- mongolog:add_command(is_list).
:- mongolog:add_command(compound).

%% ground(@Term)
% True if Term holds no free variables. 
%
mongolog:step_compile(ground(Arg), _Ctx, []) :-
	% argument is grounded already compile-time
	ground(Arg), !.

mongolog:step_compile(ground(Arg), Ctx, []) :-
	% argument was not referred to before in query, thus cannot be ground
	term_variables(Arg,Vars),
	member(Var,Vars),
	\+ mongolog:is_referenced(Var,Ctx), !,
	fail.

mongolog:step_compile(ground(Arg), Ctx, Pipeline) :-
	mongolog:var_key_or_val(Arg, Ctx, Arg0),
	findall(Step,
		(	Step=['$set', ['t_term', Arg0]]
		% fail if t_term is a variable
		;	Step=['$match', ['t_term.type', ['$ne', string('var')]]]
		% fail if t_term is a term with a variable in arguments
		;	Step=['$set', ['t_num_vars', ['$cond', [
					['if', ['$not', array([string('$t_term.value.args')])]],
					['then', integer(0)],
					['else', ['$reduce', [
						['input', string('$t_term.value.args')],
						['initialValue', integer(0)],
						['in', ['$cond', [
							['if', ['$ne', array([string('$$this.type'), string('var')])]],
							['then', string('$$value')],
							['else', ['$add', array([string('$$value'), integer(1)])]]
						]]]
					]]]
			]]]]
		;	Step=['$match', ['t_num_vars', integer(0)]]
		% cleanup
		;	Step=['$unset', array([string('t_term'), string('t_num_vars')])]
		),
		Pipeline).

%% var(@Term)
% True if Term currently is a free variable.
%
mongolog:step_compile(var(Arg), _Ctx, []) :-
	% argument is nonvar already compile-time, thus cannot be var
	nonvar(Arg), !,
	fail.

mongolog:step_compile(var(Arg), Ctx, []) :-
	% argument was not referred to before in query, thus must be var
	\+ mongolog:is_referenced(Arg,Ctx), !.

mongolog:step_compile(
		var(Arg), Ctx,
		[['$match', [
			[Key0, ['$eq', string('var')]]
		]]]) :-
	mongolog:var_key(Arg, Ctx, Key),
	atom_concat(Key, '.type', Key0).

%% number(@Term)
% True if Term is bound to a rational number (including integers) or a floating point number.
%
mongolog:step_compile(number(Arg), Ctx, Pipeline) :-
	match_type_(Arg, number, number, Ctx, Pipeline).

%% atom(@Term)
% True if Term is bound to an atom.
%
mongolog:step_compile(atom(Arg), Ctx, Pipeline) :-
	match_type_(Arg, atom, string, Ctx, Pipeline).

%% is_list(+Term)
%
%
mongolog:step_compile(is_list(Arg), Ctx, Pipeline) :-
	match_type_(Arg, is_list, array, Ctx, Pipeline).

%% compound(@Term) [ISO]
% True if Term is bound to a compound term.
%
mongolog:step_compile(compound(Arg), _Ctx, []) :-
	% argument is nonvar already compile-time
	nonvar(Arg), !,
	compound(Arg).

mongolog:step_compile(compound(Arg), Ctx, []) :-
	% argument was not referred to before in query, thus cannot be compound
	\+ mongolog:is_referenced(Arg,Ctx), !,
	fail.

mongolog:step_compile(
		compound(Arg), Ctx,
		[['$match', [
			[Key0, ['$exists', bool(true)]]
		]]]) :-
	% compound terms are represented as documents that have
	% a field "functor"
	mongolog:var_key(Arg, Ctx, Key),
	atom_concat(Key,'.value.functor',Key0).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
match_type_(Arg, Goal, _Type, _Ctx, []) :-
	% argument is grounded already compile-time
	ground(Arg), !,
	call(Goal, Arg).

match_type_(Arg, _, _Type, Ctx, []) :-
	% argument was not referred to before in query, so cannot be ground
	\+ mongolog:is_referenced(Arg,Ctx), !,
	fail.

match_type_(Arg, _Goal, Type, Ctx,
		[['$match',
			[Key, ['$type', string(Type)]]
		]]) :-
	mongolog:var_key(Arg, Ctx, Key).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_typecheck').

test('ground(?Term)'):-
	assert_true(lang_query:test_command(
		ground(Number), Number, 4.5)),
	assert_true(lang_query:test_command(
		ground(foo(Number)), Number, 4.5)),
	assert_false(lang_query:test_command(
		ground(_), _, 4.5)),
	assert_false(lang_query:test_command(
		ground(foo(Number,_)), Number, 4.5)).

test('var(?Term)'):-
	assert_true(lang_query:test_command(
		var(_), _, 4.5)),
	assert_false(lang_query:test_command(
		var(Number), Number, 4.5)).

test('number(+Number)'):-
	assert_true(lang_query:test_command(
		number(Number), Number, 4.5)),
	assert_false(lang_query:test_command(
		number(Number), Number, 'foo')),
	assert_false(lang_query:test_command(
		number(Number), Number, '4.5')).

test('atom(+Atom)'):-
	assert_true(lang_query:test_command(
		atom(Atom), Atom, '4.5')),
	assert_false(lang_query:test_command(
		atom(Atom), Atom, 4.5)).

test('is_list(+List)'):-
	assert_true(lang_query:test_command(
		is_list(List), List, ['4.5'])),
	assert_false(lang_query:test_command(
		is_list(List), List, 4.5)).

test('compound(+Term)'):-
	assert_true(lang_query:test_command(
		compound(Term), Term, foo('4.5'))),
	assert_false(lang_query:test_command(
		compound(Term), Term, 4.5)).

:- end_tests('mongolog_typecheck').
