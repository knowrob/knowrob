:- module(mongolog_atoms, []).
/** <module> Analysing and constructing atoms in mongolog programs.

The following predicates are supported:

| Predicate             | Arguments |
| ---                   | ---       |
| atom_number/2         | ?Atom, ?Number |
| atom_length/2         | +Atom, ?Length |
| atom_prefix/2         | +Atom, +Prefix |
| atom_concat/3         | ?Atom1, ?Atom2, ?Atom3 |
| atomic_list_concat/2  | +List, ?Atom |
| atomic_list_concat/3  | +List, +Separator, ?Atom |
| upcase_atom/2         | +AnyCase, ?UpperCase |
| downcase_atom/2       | +AnyCase, ?LowerCase |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=manipatom
@license BSD
*/

:- use_module('mongolog').

%% query commands
:- mongolog:add_command(atom_number).
:- mongolog:add_command(atom_length).
:- mongolog:add_command(atom_prefix).
:- mongolog:add_command(atom_concat).
:- mongolog:add_command(atomic_list_concat).
:- mongolog:add_command(upcase_atom).
:- mongolog:add_command(downcase_atom).

%% query compilation
mongolog:step_compile(
		atom_number(Atom,Number), _, []) :-
	(ground(Atom);ground(Number)),!,
	atom_number(Atom,Number).

mongolog:step_compile(
		atom_number(Atom,Number),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	mongolog:var_key_or_val(Number,Ctx,Number0),
	findall(Step,
		(	mongolog:set_if_var(Atom,    ['$toString', Number0], Ctx, Step)
		;	mongolog:set_if_var(Number,  ['$toDouble', Atom0],   Ctx, Step)
		;	mongolog:match_equals(Atom0, ['$toString', Number0], Step)
		),
		Pipeline).

mongolog:step_compile(atom_length(Atom,Length), _, []) :-
	ground(Atom),!,
	atom_length(Atom,Length).

mongolog:step_compile(
		atom_length(Atom,Length),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	mongolog:var_key_or_val(Length,Ctx,Length0),
	findall(Step,
		(	mongolog:set_if_var(Length,    ['$strLenCP', Atom0], Ctx, Step)
		;	mongolog:match_equals(Length0, ['$strLenCP', Atom0], Step)
		),
		Pipeline).

mongolog:step_compile(upcase_atom(Atom,UpperCase), _, []) :-
	ground(Atom),!,
	upcase_atom(Atom,UpperCase).

mongolog:step_compile(
		upcase_atom(Atom,UpperCase),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	mongolog:var_key_or_val(UpperCase,Ctx,UpperCase0),
	findall(Step,
		(	mongolog:set_if_var(UpperCase,    ['$toUpper', Atom0], Ctx, Step)
		;	mongolog:match_equals(UpperCase0, ['$toUpper', Atom0], Step)
		),
		Pipeline).

mongolog:step_compile(downcase_atom(Atom,UpperCase), _, []) :-
	ground(Atom),!,
	downcase_atom(Atom,UpperCase).

mongolog:step_compile(
		downcase_atom(Atom,LowerCase),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	mongolog:var_key_or_val(LowerCase,Ctx,LowerCase0),
	findall(Step,
		(	mongolog:set_if_var(LowerCase,    ['$toLower', Atom0], Ctx, Step)
		;	mongolog:match_equals(LowerCase0, ['$toLower', Atom0], Step)
		),
		Pipeline).

mongolog:step_compile(atom_prefix(Atom,Prefix), _, []) :-
	ground([Atom,Prefix]),!,
	atom_prefix(Atom,Prefix).

mongolog:step_compile(
		atom_prefix(Atom,Prefix),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	mongolog:var_key_or_val(Prefix,Ctx,Prefix0),
	findall(Step,
		mongolog:match_equals(Prefix0,
			['$substr', array([
				Atom0, int(0), ['$strLenCP', Prefix0]
			])],
			Step
		),
		Pipeline).

mongolog:step_compile(atom_concat(Left,Right,Atom), _, []) :-
	ground(Left),
	ground(Right),!,
	atom_concat(Left,Right,Atom).

mongolog:step_compile(
		atom_concat(Left,Right,Atom),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(Left,Ctx,Left0),
	mongolog:var_key_or_val(Right,Ctx,Right0),
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	findall(Step,
		(	mongolog:set_if_var(Left, ['$substr', array([Atom0,
				int(0),
				['$subtract', array([ ['$strLenCP',Atom0], ['$strLenCP',Right0] ])]
			])], Ctx, Step)
		;	mongolog:set_if_var(Right, ['$substr', array([Atom0,
				['$strLenCP',Left0],
				['$strLenCP',Atom0]
			])], Ctx, Step)
		;	mongolog:set_if_var(Atom,    ['$concat', array([Left0,Right0])], Ctx, Step)
		;	mongolog:match_equals(Atom0, ['$concat', array([Left0,Right0])], Step)
		),
		Pipeline).

mongolog:step_compile(atomic_list_concat(List, Atom), _, []) :-
	ground(List),!,
	atomic_list_concat(List, Atom).

mongolog:step_compile(
		atomic_list_concat(List, Atom),
		Ctx, Pipeline) :-
	findall(Resolved,
		(	member(Unresolved,List),
			mongolog:var_key_or_val(Unresolved,Ctx,Resolved)
		),
		List0),
	mongolog:var_key_or_val(Atom,Ctx,Atom0),
	findall(Step,
		(	mongolog:set_if_var(Atom,    ['$concat', array(List0)], Ctx, Step)
		;	mongolog:match_equals(Atom0, ['$concat', array(List0)], Step)
		),
		Pipeline).

mongolog:step_compile(atomic_list_concat(List, Sep, Atom), _, []) :-
	ground(Sep),
	(ground(List);ground(Atom)),!,
	atomic_list_concat(List, Sep, Atom).

mongolog:step_compile(
		atomic_list_concat(List, Sep, Atom),
		Ctx, Pipeline) :-
	findall(Resolved,
		(	member(Unresolved,List),
			mongolog:var_key_or_val(Unresolved,Ctx,Resolved)
		),
		List0),
	mongolog:var_key_or_val(Sep, Ctx, Sep0),
	mongolog:var_key_or_val(Atom, Ctx, Atom0),
	add_separator(List0, Sep0, List1),
	findall(Step,
		(	mongolog:set_if_var(Atom,    ['$concat', array(List1)], Ctx, Step)
		;	mongolog:match_equals(Atom0, ['$concat', array(List1)], Step)
		),
		Pipeline).

%%
add_separator([], _, []) :- !.
add_separator([X], _, [X]) :- !.
add_separator([X0,X1|Xs], Sep, [X0,Sep,X1|Ys]) :-
	add_separator(Xs, Sep, Ys).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_atoms').

test('atom_number(+Atom,-Num)'):-
	mongolog:test_call(
		atom_number(Atom, Num), Atom, '4.5'),
	assert_equals(Num, 4.5).

test('atom_number(+Atom,+Num)'):-
	assert_true(mongolog:test_call(
		atom_number(Atom, 4.5), Atom, '4.5')),
	assert_true(mongolog:test_call(
		atom_number(Atom, -2.25), Atom, '-2.25')).

test('atom_number(NaN,_)', [throws(mng_error(_))]):-
	mongolog:test_call(
		atom_number(Atom,_), Atom, 'not a number').

test('atom_length(+Atom,-Length)'):-
	mongolog:test_call(
		atom_length(Atom, Len), Atom, '4.5'),
	assert_equals(Len, 3).

test('upcase_atom(+Atom,+Uppercase)'):-
	assert_true(mongolog:test_call(
		upcase_atom(Atom, 'FOO 3'), Atom, 'foo 3')),
	assert_true(mongolog:test_call(
		upcase_atom(Atom, 'FOO BAR'), Atom, 'foo BAR')),
	assert_true(mongolog:test_call(
		upcase_atom(Atom, ''), Atom, '')),
	assert_false(mongolog:test_call(
		upcase_atom(Atom, 'Foo Bar'), Atom, 'foo BAR')).

test('upcase_atom(+Atom,-Uppercase)'):-
	mongolog:test_call(
		upcase_atom(Atom, Uppercase), Atom, 'foo Bar'),
	assert_equals(Uppercase, 'FOO BAR').

test('downcase_atom(+Atom,+Uppercase)'):-
	assert_true(mongolog:test_call(
		downcase_atom(Atom, 'foo 3'), Atom, 'foo 3')),
	assert_true(mongolog:test_call(
		downcase_atom(Atom, 'foo bar'), Atom, 'foo BAR')),
	assert_true(mongolog:test_call(
		downcase_atom(Atom, ''), Atom, '')),
	assert_false(mongolog:test_call(
		downcase_atom(Atom, 'Foo Bar'), Atom, 'foo BAR')).

test('atom_length(+Atom,+Length)'):-
	assert_true(mongolog:test_call(
		atom_length(Atom, 3), Atom, 'foo')),
	assert_false(mongolog:test_call(
		atom_length(Atom, 2), Atom, 'foo')),
	assert_true(mongolog:test_call(
		atom_length(Atom, 0), Atom, '')).

test('atom_prefix(+Atom,+Prefix)'):-
	assert_true(mongolog:test_call(
		atom_prefix(Atom, 'f'), Atom, 'foo')),
	assert_true(mongolog:test_call(
		atom_prefix(Atom, 'fo'), Atom, 'foo')),
	assert_true(mongolog:test_call(
		atom_prefix(Atom, 'foo'), Atom, 'foo')),
	assert_false(mongolog:test_call(
		atom_prefix(Atom, 'bar'), Atom, 'foo')).

test('atom_concat(+A1,+A2,+A3)'):-
	assert_true(mongolog:test_call(
		atom_concat('foo', 'bar', Atom),
		Atom, 'foobar')).

test('atom_concat(+A1,+A2,-A3)'):-
	mongolog:test_call(
		atom_concat(A1, 'bar', A3),
		A1, 'foo'),
	assert_equals(A3, 'foobar').

test('atom_concat(+A1,-A2,+A3)'):-
	mongolog:test_call(
		atom_concat(A1, A2, 'foobar'),
		A1, 'foo'),
	assert_equals(A2, 'bar').

test('atom_concat(-A1,+A2,+A3)'):-
	mongolog:test_call(
		atom_concat(A1, A2, 'foobar'),
		A2, 'bar'),
	assert_equals(A1, 'foo').

test('atomic_list_concat(+List,+Atom)'):-
	assert_true(mongolog:test_call(
		atomic_list_concat(['foo', 'bar'], Atom),
		Atom, 'foobar')).

test('atomic_list_concat(+List,-Atom)'):-
	mongolog:test_call(
		atomic_list_concat([X1, 'bar'], Atom),
		X1, 'foo'),
	assert_equals(Atom, 'foobar').

test('atomic_list_concat(+List,+Sep,-Atom)'):-
	mongolog:test_call(
		atomic_list_concat([X1, 'bar'], '-', Atom),
		X1, 'foo'),
	assert_equals(Atom, 'foo-bar').

:- end_tests('mongolog_atoms').
