:- module(lang_comparison, []).

:- use_module(library('lang/compiler')).

%% query_compiler:add_command
:- query_compiler:add_command( ==, [ask]).
:- query_compiler:add_command(\==, [ask]).


%% query_compiler:step_var
query_compiler:step_var( ==(X,Y), Ctx, Var) :- query_compiler:get_var([X,Y],Ctx,Var).
query_compiler:step_var(\==(X,Y), Ctx, Var) :- query_compiler:get_var([X,Y],Ctx,Var).


%% @Term1 == @Term2
% True if Term1 is equivalent to Term2. A variable is only identical to a sharing variable
%
query_compiler:step_compile(==(X,Y), _, []) :-
	ground([X,Y]),!,
	X == Y.

query_compiler:step_compile(==(X,Y), Ctx, [
		['$set',   ['t_equal', ['$eq', array([X0,Y0])]]],
		['$match', ['t_equal', bool(true)]],
		['$unset', string('t_equal')]
	]) :-
	% FIXME: unified variables are thought to be equal, eg.
	%			?- A=B, foo(A) == foo(B).
	%
	query_compiler:var_key_or_val(X,Ctx,X0),
	query_compiler:var_key_or_val(Y,Ctx,Y0).

%% @Term1 \== @Term2
% Equivalent to \+Term1 == Term2.
%
query_compiler:step_compile(\==(X,Y), _, []) :-
	ground([X,Y]),!,
	X \== Y.

query_compiler:step_compile(\==(X,Y), Ctx, [
		['$set',   ['t_equal', ['$eq', array([X0,Y0])]]],
		['$match', ['t_equal', bool(false)]],
		['$unset', string('t_equal')]
	]) :-
	query_compiler:var_key_or_val(X,Ctx,X0),
	query_compiler:var_key_or_val(Y,Ctx,Y0).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('lang_comparison').

test('==(+Atom,+Term)'):-
	assert_true(lang_query:test_command(
		(Atom == '4.5'), Atom, '4.5')),
	assert_true(lang_query:test_command(
		(Atom == 'foo'), Atom, 'foo')),
	assert_false(lang_query:test_command(
		(Atom == 'foo'), Atom, '4.5')).

test('==(+Number,+Term)'):-
	assert_true(lang_query:test_command(
		(X == 4.5), X, 4.5)),
	assert_false(lang_query:test_command(
		(X == 4.5), X, '4.5')).
		
test('==(+Term1,+Term2)'):-
	assert_true(lang_query:test_command(
		(X == foo(2)), X, foo(2))),
	assert_true(lang_query:test_command(
		(X == foo(bar,2)), X, foo(bar,2))),
	assert_true(lang_query:test_command(
		(X == foo(Y)), X, foo(Y))),
	assert_false(lang_query:test_command(
		(X == foo(3)), X, foo(2))).

test('\\==(+Term1,+Term2)'):-
	assert_false(lang_query:test_command(
		(Atom \== 'foo'), Atom, 'foo')),
	assert_false(lang_query:test_command(
		(Atom \== 4.5), Atom, 4.5)),
	assert_true(lang_query:test_command(
		(Atom \== 'foo'), Atom, '4.5')),
	assert_true(lang_query:test_command(
		(Atom \== 4.5), Atom, '4.5')).

:- end_tests('lang_comparison').
	