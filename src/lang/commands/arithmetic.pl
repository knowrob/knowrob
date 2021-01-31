:- module(lang_arithmetic, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(is,  [ask]).
:- query_compiler:add_command(>,   [ask]).
:- query_compiler:add_command(<,   [ask]).
:- query_compiler:add_command(=<,  [ask]).
:- query_compiler:add_command(>=,  [ask]).
:- query_compiler:add_command(=\=, [ask]).
:- query_compiler:add_command(=:=, [ask]).

%% query variables
query_compiler:step_var( is(X,Y), Var) :- assignment_var(X,Y,Var).
query_compiler:step_var(  <(X,Y), Var) :- comparison_var(X,Y,Var).
query_compiler:step_var(  >(X,Y), Var) :- comparison_var(X,Y,Var).
query_compiler:step_var( =<(X,Y), Var) :- comparison_var(X,Y,Var).
query_compiler:step_var( >=(X,Y), Var) :- comparison_var(X,Y,Var).
query_compiler:step_var(=\=(X,Y), Var) :- comparison_var(X,Y,Var).
query_compiler:step_var(=:=(X,Y), Var) :- comparison_var(X,Y,Var).

%% query compilation
% TODO: early evaluation if ground at compile-time!
%
query_compiler:step_compile( is(X,Y), _, Z) :- assignment(X,Y,Z).
query_compiler:step_compile(  <(X,Y), _, Z) :- comparison(  <(X,Y),Z).
query_compiler:step_compile( =<(X,Y), _, Z) :- comparison( =<(X,Y),Z).
query_compiler:step_compile(  >(X,Y), _, Z) :- comparison(  >(X,Y),Z).
query_compiler:step_compile( >=(X,Y), _, Z) :- comparison( >=(X,Y),Z).
query_compiler:step_compile(=\=(X,Y), _, Z) :- comparison(=\=(X,Y),Z).
query_compiler:step_compile(=:=(X,Y), _, Z) :- comparison(=:=(X,Y),Z).

%%
assignment_var(Var, _Exp, [VarKey, Var]) :-
	query_compiler:var_key(Var, VarKey).
assignment_var(_Var, Exp, [VarKey, Var]) :-
	expression_var(Exp, VarKey, Var).

%%
comparison_var(Exp1, Exp2, [VarKey, Var]) :-
	(	expression_var(Exp1, VarKey, Var)
	;	expression_var(Exp2, VarKey, Var)
	).

%%
expression_var(Exp, Key, Var) :-
	term_variables(Exp, ExpVars),
	member(Var, ExpVars),
	query_compiler:var_key(Var, Key).

%% $set var to evaluated number
assignment(Var, Exp, []) :-
	ground(Exp),!,
	Var is Exp.

assignment(Var, Exp, [['$set', [Varkey, Doc]]]) :-
	% FIXME: SWI Prolog allows to write e.g. `7 is 7`,
	%           so this needs to be expanded into $set + $match pipeline
	query_compiler:var_key(Var, Varkey),
	expression(Exp,Doc).

%% compare two expressions
comparison(Exp, []) :-
	ground(Exp),!,
	call(Exp).

comparison(Exp, [
		['$set',   ['t_success', [MngOperator, array([Doc1,Doc2])]]],
		['$match', ['t_success', bool(true)]],
		['$unset', string('t_success')]
	]) :-
	Exp =.. [PlOperator, Left, Right],
	expression_operator(PlOperator, MngOperator),
	expression(Left, Doc1),
	expression(Right, Doc2).

%% variables
expression(Var, string(VarValue)) :-
	var(Var),!,
	query_compiler:var_key(Var,Key),
	atom_concat('$',Key,VarValue).
%% functions
expression(Exp, Doc) :-
	compound(Exp),!,
	Exp =.. [Functor|Args],
	expression_function(Functor, Operator),
	maplist(expression, Args, SubDocs),
	(	SubDocs=[Single]
	->	Doc=[Operator, Single]
	;	Doc=[Operator, array(SubDocs)]
	).
%% constant values
expression(Val, double(Val)) :- number(Val),!.
%% named constants
expression(pi,      double(V))          :- V is pi,!.
expression(e,       double(V))          :- V is e,!.
expression(epsilon, double(V))          :- V is epsilon,!.
expression(inf,     double('Infinity')) :- !.
expression(nan,     double('NaN'))      :- !.

%% arithmetic operators
expression_operator(  <, '$lt').
expression_operator( =<, '$lte').
expression_operator(  >, '$gt').
expression_operator( >=, '$gte').
expression_operator(=\=, '$ne').
expression_operator(=:=, '$eq').

%% arithmetic functions
expression_function(abs,      '$abs').
expression_function(round,    '$round').
expression_function(floor,    '$floor').
expression_function(truncate, '$trunc').
expression_function(ceil,     '$ceil').
expression_function(ceiling,  '$ceil').
expression_function(sqrt,     '$sqrt').
expression_function(exp,      '$exp').
expression_function(log,      '$ln').
expression_function(log10,    '$log10').
expression_function(mod,      '$mod').
expression_function(sin,      '$sin').
expression_function(cos,      '$cos').
expression_function(tan,      '$tan').
expression_function(asin,     '$asin').
expression_function(acos,     '$acos').
expression_function(atan,     '$atan').
expression_function(atan2,    '$atan2').
expression_function(max,      '$max').
expression_function(min,      '$min').
expression_function(+,        '$add').
expression_function(-,        '$subtract').
expression_function(/,        '$divide').
expression_function(*,        '$multiply').
%expression_function(_,'$sum').
%expression_function(_,'$avg').

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('lang_arithmetic').

test('var is constant'):-
	lang_query:test_command(
		(Y is X), X, double(-3.25)),
	assert_equals(Y, -3.25).

test('var is exp'):-
	lang_query:test_command(
		(Y is (X + 0.5)*2), X, double(2.5)),
	assert_equals(Y, 6.0).

test('constant is constant', [fixme('left-side of is/2 cannot be ground')]):-
	assert_true(lang_query:test_command(
		(3.0 is X), X, double(-3))),
	assert_false(lang_query:test_command(
		(4.0 is X), X, double(-3))).

test('var < constant'):-
	assert_true(lang_query:test_command(
		(X < 7.0), X, double(6))),
	assert_false(lang_query:test_command(
		(X < 7.0), X, double(8))),
	assert_false(lang_query:test_command(
		(X < 7.0), X, double(7))).

test('exp < exp'):-
	assert_true(lang_query:test_command(
		((X*2) < (X + 5)), X, double(2))),
	assert_false(lang_query:test_command(
		((X*2) < (X + 5)), X, double(5))),
	assert_false(lang_query:test_command(
		((X*2) < (X + 5)), X, double(6))).

:- end_tests('lang_arithmetic').
