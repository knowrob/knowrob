:- module(lang_atoms, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(atom_number,         [ask]).
:- query_compiler:add_command(atom_length,         [ask]).
:- query_compiler:add_command(atom_prefix,         [ask]).
:- query_compiler:add_command(atom_concat,         [ask]).
:- query_compiler:add_command(atomic_list_concat,  [ask]).

%% query variables
query_compiler:step_var(atom_number(A,N), Ctx, Var)      :- query_compiler:get_var([A,N],Ctx,Var).
query_compiler:step_var(atom_length(A,L), Ctx, Var)      :- query_compiler:get_var([A,L],Ctx,Var).
query_compiler:step_var(atom_prefix(A,P), Ctx, Var)      :- query_compiler:get_var([A,P],Ctx,Var).
query_compiler:step_var(atom_concat(A1,A2,A3), Ctx, Var) :- query_compiler:get_var([A1,A2,A3],Ctx,Var).
query_compiler:step_var(atomic_list_concat(List,Sep,A), Ctx, Var) :- query_compiler:get_var([A,Sep|List],Ctx,Var).
query_compiler:step_var(atomic_list_concat(List,A), Ctx, Var)     :- query_compiler:get_var([A|List],Ctx,Var).

%% query compilation
query_compiler:step_compile(
		atom_number(Atom,Number), _,
		[]) :-
	(ground(Atom);ground(Number)),!,
	atom_number(Atom,Number).

query_compiler:step_compile(
		atom_number(Atom,Number),
		Ctx, Pipeline) :-
	query_compiler:var_key_or_val(Atom,Ctx,Atom0),
	query_compiler:var_key_or_val(Number,Ctx,Number0),
	findall(Step,
		(	query_compiler:set_if_var(Atom,    ['$toString', Number0], Ctx, Step)
		;	query_compiler:set_if_var(Number,  ['$toDouble', Atom0],   Ctx, Step)
		;	query_compiler:match_equals(Atom0, ['$toString', Number0], Step)
		),
		Pipeline).

query_compiler:step_compile(atom_length(Atom,Length), _, []) :-
	ground(Atom),!,
	atom_length(Atom,Length).

query_compiler:step_compile(
		atom_length(Atom,Length),
		Ctx, Pipeline) :-
	query_compiler:var_key_or_val(Atom,Ctx,Atom0),
	query_compiler:var_key_or_val(Length,Ctx,Length0),
	findall(Step,
		(	query_compiler:set_if_var(Length,    ['$strLenCP', Atom0], Ctx, Step)
		;	query_compiler:match_equals(Length0, ['$strLenCP', Atom0], Step)
		),
		Pipeline).

query_compiler:step_compile(atom_prefix(Atom,Prefix), _, []) :-
	ground([Atom,Prefix]),!,
	atom_prefix(Atom,Prefix).

query_compiler:step_compile(
		atom_prefix(Atom,Prefix),
		Ctx, Pipeline) :-
	% FIXME: SWI Prolog allows atom(Atom), var(Prefix), and then
	%         yields all possible prefixes.
	query_compiler:var_key_or_val(Atom,Ctx,Atom0),
	query_compiler:var_key_or_val(Prefix,Ctx,Prefix0),
	findall(Step,
		query_compiler:match_equals(Prefix0,
			['$substr', array([
				Atom0, int(0), ['$strLenCP', Prefix0]
			])],
			Step
		),
		Pipeline).

query_compiler:step_compile(atom_concat(Left,Right,Atom), _, []) :-
	ground(Left),
	ground(Right),!,
	atom_concat(Left,Right,Atom).

query_compiler:step_compile(
		atom_concat(Left,Right,Atom),
		Ctx, Pipeline) :-
	% FIXME: SWI Prolog allows var(Left), var(Right), atom(Atom), and then
	%         yields all possible concatenations.
	query_compiler:var_key_or_val(Left,Ctx,Left0),
	query_compiler:var_key_or_val(Right,Ctx,Right0),
	query_compiler:var_key_or_val(Atom,Ctx,Atom0),
	findall(Step,
		(	query_compiler:set_if_var(Left, ['$substr', array([Atom0,
				int(0),
				['$subtract', array([ ['$strLenCP',Atom0], ['$strLenCP',Right0] ])]
			])], Ctx, Step)
		;	query_compiler:set_if_var(Right, ['$substr', array([Atom0,
				['$strLenCP',Left0],
				['$strLenCP',Atom0]
			])], Ctx, Step)
		;	query_compiler:set_if_var(Atom,    ['$concat', array([Left0,Right0])], Ctx, Step)
		;	query_compiler:match_equals(Atom0, ['$concat', array([Left0,Right0])], Step)
		),
		Pipeline).

query_compiler:step_compile(atomic_list_concat(List, Atom), _, []) :-
	ground(List),!,
	atomic_list_concat(List, Atom).

query_compiler:step_compile(
		atomic_list_concat(List, Atom),
		Ctx, Pipeline) :-
	findall(Resolved,
		(	member(Unresolved,List),
			query_compiler:var_key_or_val(Unresolved,Ctx,Resolved)
		),
		List0),
	query_compiler:var_key_or_val(Atom,Ctx,Atom0),
	findall(Step,
		(	query_compiler:set_if_var(Atom,    ['$concat', array(List0)], Ctx, Step)
		;	query_compiler:match_equals(Atom0, ['$concat', array(List0)], Step)
		),
		Pipeline).

query_compiler:step_compile(atomic_list_concat(List, Sep, Atom), _, []) :-
	ground(Sep),
	(ground(List);ground(Atom)),!,
	atomic_list_concat(List, Sep, Atom).

query_compiler:step_compile(
		atomic_list_concat(List, Sep, Atom),
		Ctx, Pipeline) :-
	findall(Resolved,
		(	member(Unresolved,List),
			query_compiler:var_key_or_val(Unresolved,Ctx,Resolved)
		),
		List0),
	query_compiler:var_key_or_val(Sep, Ctx, Sep0),
	query_compiler:var_key_or_val(Atom, Ctx, Atom0),
	add_separator(List0, Sep0, List1),
	findall(Step,
		(	query_compiler:set_if_var(Atom,    ['$concat', array(List1)], Ctx, Step)
		;	query_compiler:match_equals(Atom0, ['$concat', array(List1)], Step)
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

:- begin_tests('lang_atoms').

test('atom_number(+Atom,-Num)'):-
	lang_query:test_command(
		atom_number(Atom, Num),
		Atom, string('4.5')),
	assert_equals(Num, 4.5).

test('atom_number(+Atom,+Num)'):-
	assert_true(lang_query:test_command(
		atom_number(Atom, 4.5),
		Atom, string('4.5'))),
	assert_true(lang_query:test_command(
		atom_number(Atom, -2.25),
		Atom, string('-2.25'))).

test('atom_number(NaN,_)') :- %, [throws(failed_assertion(_,_))]):-
	assert_false(lang_query:test_command(
		atom_number(Atom,_),
		Atom, string('not a number'))).

test('atom_length(+Atom,-Length)'):-
	lang_query:test_command(
		atom_length(Atom, Len),
		Atom, string('4.5')),
	assert_equals(Len, 3).

test('atom_length(+Atom,+Length)'):-
	assert_true(lang_query:test_command(
		atom_length(Atom, 3),
		Atom, string('foo'))),
	assert_false(lang_query:test_command(
		atom_length(Atom, 2),
		Atom, string('foo'))),
	assert_true(lang_query:test_command(
		atom_length(Atom, 0),
		Atom, string(''))).

test('atom_prefix(+Atom,+Prefix)'):-
	assert_true(lang_query:test_command(
		atom_prefix(Atom, 'f'),
		Atom, string('foo'))),
	assert_true(lang_query:test_command(
		atom_prefix(Atom, 'fo'),
		Atom, string('foo'))),
	assert_true(lang_query:test_command(
		atom_prefix(Atom, 'foo'),
		Atom, string('foo'))),
	assert_false(lang_query:test_command(
		atom_prefix(Atom, 'bar'),
		Atom, string('foo'))).

test('atom_concat(+A1,+A2,+A3)'):-
	assert_true(lang_query:test_command(
		atom_concat('foo', 'bar', Atom),
		Atom, string('foobar'))).

test('atom_concat(+A1,+A2,-A3)'):-
	lang_query:test_command(
		atom_concat(A1, 'bar', A3),
		A1, string('foo')),
	assert_equals(A3, 'foobar').

test('atom_concat(+A1,-A2,+A3)'):-
	lang_query:test_command(
		atom_concat(A1, A2, 'foobar'),
		A1, string('foo')),
	assert_equals(A2, 'bar').

test('atom_concat(-A1,+A2,+A3)'):-
	lang_query:test_command(
		atom_concat(A1, A2, 'foobar'),
		A2, string('bar')),
	assert_equals(A1, 'foo').

test('atomic_list_concat(+List,+Atom)'):-
	assert_true(lang_query:test_command(
		atomic_list_concat(['foo', 'bar'], Atom),
		Atom, string('foobar'))).

test('atomic_list_concat(+List,-Atom)'):-
	lang_query:test_command(
		atomic_list_concat([X1, 'bar'], Atom),
		X1, string('foo')),
	assert_equals(Atom, 'foobar').

test('atomic_list_concat(+List,+Sep,-Atom)'):-
	lang_query:test_command(
		atomic_list_concat([X1, 'bar'], '-', Atom),
		X1, string('foo')),
	assert_equals(Atom, 'foo-bar').

:- end_tests('lang_atoms').
