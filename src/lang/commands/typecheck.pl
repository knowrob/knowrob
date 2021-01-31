:- module(typecheck_commands, []).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(ground,  [ask,tell]).
:- query_compiler:add_command(var,     [ask,tell]).
:- query_compiler:add_command(number,  [ask,tell]).
:- query_compiler:add_command(atom,    [ask,tell]).
:- query_compiler:add_command(is_list, [ask,tell]).

%%
query_compiler:step_var(ground(X),  [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(var(X),     [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(number(X),  [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(atom(X),    [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(is_list(X), [V,X]) :- query_compiler:var_key(X, V).

%%
% ground(X) holds iff a previous step has assigned
% a value to the variable key.
%
query_compiler:step_compile(ground(Arg), _Context, []) :-
	% argument is grounded already compile-time
	ground(Arg), !.

query_compiler:step_compile(ground(Arg), Context, []) :-
	% argument was not referred to before in query, thus cannot be ground
	\+ query_compiler:is_referenced(Arg,Context), !,
	fail.

query_compiler:step_compile(
		ground(Arg), _Context,
		[['$match', [
			[Key, ['$exists', bool(true)]]
		]]]) :-
	query_compiler:var_key(Arg, Key).

%%
% var(X) holds iff no previous step has assigned
% a value to the variable key.
%
query_compiler:step_compile(var(Arg), _Context, []) :-
	% argument is grounded already compile-time, thus cannot be var
	ground(Arg), !,
	fail.

query_compiler:step_compile(var(Arg), Context, []) :-
	% argument was not referred to before in query, thus must be var
	\+ query_compiler:is_referenced(Arg,Context), !.

query_compiler:step_compile(
		var(Arg), _Context,
		[['$match', [
			[Key, ['$exists', bool(false)]]
		]]]) :-
	query_compiler:var_key(Arg, Key).

%%
%
%
query_compiler:step_compile(number(Arg), _Context, []) :-
	% argument is grounded already compile-time
	ground(Arg), !,
	number(Arg).

query_compiler:step_compile(number(Arg), Context, []) :-
	% argument was not referred to before in query, thus cannot be number
	\+ query_compiler:is_referenced(Arg,Context), !,
	fail.

query_compiler:step_compile(number(Arg), Context, Pipeline) :-
	match_type_(Arg, number, number, Context, Pipeline).

%%
%
%
query_compiler:step_compile(atom(Arg), Context, Pipeline) :-
	match_type_(Arg, atom, string, Context, Pipeline).

%%
%
%
query_compiler:step_compile(is_list(Arg), Context, Pipeline) :-
	match_type_(Arg, is_list, array, Context, Pipeline).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
match_type_(Arg, Goal, _Type, _Context, []) :-
	% argument is grounded already compile-time
	ground(Arg), !,
	call(Goal, [Arg]).

match_type_(Arg, _, _Type, Context, []) :-
	% argument was not referred to before in query, so cannot be ground
	\+ query_compiler:is_referenced(Arg,Context), !,
	fail.

match_type_(Arg, _Goal, Type, _Context,
		[['$match',
			[Key, ['$type', string(Type)]]
		]]) :-
	query_compiler:var_key(Arg, Key).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('typecheck_commands').

test('ground(?Term)'):-
	assert_true(lang_query:test_command(
		ground(Number), Number, double(4.5))),
	assert_false(lang_query:test_command(
		ground(_), _, double(4.5))).

test('var(?Term)'):-
	assert_true(lang_query:test_command(
		var(_), _, double(4.5))),
	assert_false(lang_query:test_command(
		var(Number), Number, double(4.5))).

test('number(+Number)'):-
	assert_true(lang_query:test_command(
		number(Number), Number, double(4.5))),
	assert_false(lang_query:test_command(
		number(Number), Number, string('foo'))),
	assert_false(lang_query:test_command(
		number(Number), Number, string('4.5'))).

test('atom(+Atom)'):-
	assert_true(lang_query:test_command(
		atom(Atom), Atom, string('4.5'))),
	assert_false(lang_query:test_command(
		atom(Atom), Atom, double(4.5))).

test('is_list(+List)'):-
	assert_true(lang_query:test_command(
		is_list(List), List, array([string('4.5')]))),
	assert_false(lang_query:test_command(
		is_list(List), List, double(4.5))).

:- end_tests('typecheck_commands').
