:- module(typecheck_commands, []).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(ground,   [ask,tell]).
:- query_compiler:add_command(var,      [ask,tell]).
:- query_compiler:add_command(number,   [ask,tell]).
:- query_compiler:add_command(atom,     [ask,tell]).
:- query_compiler:add_command(is_list,  [ask,tell]).
:- query_compiler:add_command(compound, [ask,tell]).

%%
query_compiler:step_var(ground(X),  Ctx, [V,X]) :- query_compiler:var_key(X, Ctx, V).
query_compiler:step_var(var(X),     Ctx, [V,X]) :- query_compiler:var_key(X, Ctx, V).
query_compiler:step_var(number(X),  Ctx, [V,X]) :- query_compiler:var_key(X, Ctx, V).
query_compiler:step_var(atom(X),    Ctx, [V,X]) :- query_compiler:var_key(X, Ctx, V).
query_compiler:step_var(is_list(X), Ctx, [V,X]) :- query_compiler:var_key(X, Ctx, V).
query_compiler:step_var(compound(X), Ctx, Var)  :- query_compiler:get_var(X, Ctx, Var).

%%
% ground(X) holds iff a previous step has assigned
% a value to the variable key.
%
query_compiler:step_compile(ground(Arg), _Ctx, []) :-
	% argument is grounded already compile-time
	ground(Arg), !.

query_compiler:step_compile(ground(Arg), Ctx, []) :-
	% argument was not referred to before in query, thus cannot be ground
	term_variables(Arg,Vars),
	member(Var,Vars),
	\+ query_compiler:is_referenced(Var,Ctx), !,
	fail.

query_compiler:step_compile(ground(Arg), Ctx, Pipeline) :-
	query_compiler:var_key_or_val(Arg, Ctx, Arg0),
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

%%
% var(X) holds iff no previous step has assigned
% a value to the variable key.
%
query_compiler:step_compile(var(Arg), _Ctx, []) :-
	% argument is nonvar already compile-time, thus cannot be var
	nonvar(Arg), !,
	fail.

query_compiler:step_compile(var(Arg), Ctx, []) :-
	% argument was not referred to before in query, thus must be var
	\+ query_compiler:is_referenced(Arg,Ctx), !.

query_compiler:step_compile(
		var(Arg), Ctx,
		[['$match', [
			[Key0, ['$eq', string('var')]]
		]]]) :-
	query_compiler:var_key(Arg, Ctx, Key),
	atom_concat(Key, '.type', Key0).

%%
%
%
query_compiler:step_compile(number(Arg), Ctx, Pipeline) :-
	match_type_(Arg, number, number, Ctx, Pipeline).

%%
%
%
query_compiler:step_compile(atom(Arg), Ctx, Pipeline) :-
	match_type_(Arg, atom, string, Ctx, Pipeline).

%%
%
%
query_compiler:step_compile(is_list(Arg), Ctx, Pipeline) :-
	match_type_(Arg, is_list, array, Ctx, Pipeline).

%% compound(@Term) [ISO]
% True if Term is bound to a compound term.
%
query_compiler:step_compile(compound(Arg), _Ctx, []) :-
	% argument is nonvar already compile-time
	nonvar(Arg), !,
	compound(Arg).

query_compiler:step_compile(compound(Arg), Ctx, []) :-
	% argument was not referred to before in query, thus cannot be compound
	\+ query_compiler:is_referenced(Arg,Ctx), !,
	fail.

query_compiler:step_compile(
		compound(Arg), Ctx,
		[['$match', [
			[Key0, ['$exists', bool(true)]]
		]]]) :-
	% compound terms are represented as documents that have
	% a field "functor"
	query_compiler:var_key(Arg, Ctx, Key),
	atom_concat(Key,'.value.functor',Key0).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
match_type_(Arg, Goal, _Type, _Ctx, []) :-
	% argument is grounded already compile-time
	ground(Arg), !,
	call(Goal, [Arg]).

match_type_(Arg, _, _Type, Ctx, []) :-
	% argument was not referred to before in query, so cannot be ground
	\+ query_compiler:is_referenced(Arg,Ctx), !,
	fail.

match_type_(Arg, _Goal, Type, Ctx,
		[['$match',
			[Key, ['$type', string(Type)]]
		]]) :-
	query_compiler:var_key(Arg, Ctx, Key).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('typecheck_commands').

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

:- end_tests('typecheck_commands').
