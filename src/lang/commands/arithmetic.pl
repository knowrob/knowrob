:- module(lang_arithmetic, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(is,  [ask,tell]).
:- query_compiler:add_command(>,   [ask,tell]).
:- query_compiler:add_command(<,   [ask,tell]).
:- query_compiler:add_command(=<,  [ask,tell]).
:- query_compiler:add_command(>=,  [ask,tell]).
:- query_compiler:add_command(=\=, [ask,tell]).
:- query_compiler:add_command(=:=, [ask,tell]).
:- query_compiler:add_command(between, [ask,tell]).

%% query compilation
% TODO: early evaluation if ground at compile-time!
%
query_compiler:step_compile( is(X,Y), Ctx, Z) :- assignment(X,Y,Ctx,Z).
query_compiler:step_compile(  <(X,Y), Ctx, Z) :- comparison(  <(X,Y),Ctx,Z).
query_compiler:step_compile( =<(X,Y), Ctx, Z) :- comparison( =<(X,Y),Ctx,Z).
query_compiler:step_compile(  >(X,Y), Ctx, Z) :- comparison(  >(X,Y),Ctx,Z).
query_compiler:step_compile( >=(X,Y), Ctx, Z) :- comparison( >=(X,Y),Ctx,Z).
query_compiler:step_compile(=\=(X,Y), Ctx, Z) :- comparison(=\=(X,Y),Ctx,Z).
query_compiler:step_compile(=:=(X,Y), Ctx, Z) :- comparison(=:=(X,Y),Ctx,Z).

%% between(+Low, +High, -Value)
% Low and High are integers, High >=Low. If Value is an integer, Low =<Value =<High.
% When Value is a variable it is successively bound to all integers between Low and High. 
%
query_compiler:step_compile(
		between(Low, High, Value),
		Ctx, Pipeline) :-
	query_compiler:var_key_or_val(Low,Ctx,Low0),
	query_compiler:var_key_or_val(High,Ctx,High0),
	query_compiler:var_key_or_val(Value,Ctx,Value0),
	findall(Step,
		% TODO: conditional $set to array holding only Value if Value is given
		%       this would avoid iteration between Low and High for faster between checking.
		(	Step=['$set', ['t_index', ['$range',
				array([ Low0, ['$add', array([High0, integer(1)]) ]])
			]]]
		;	Step=['$unwind',string('$t_index')]
		;	query_compiler:set_if_var(Value, string('$t_index'), Ctx, Step)
		;	query_compiler:match_equals(Value0, string('$t_index'), Step)
		;	Step=['$unset', string('t_index')]
		),
		Pipeline).

%% $set var to evaluated number
assignment(Var, Exp, _Ctx, []) :-
	ground(Exp),!,
	Var is Exp.

assignment(Number, Exp, Ctx, Pipeline) :-
	% NOTE: SWI Prolog allows to write e.g. `7 is 7`, so here we use set+match
	query_compiler:var_key_or_val(Number,Ctx,Number0),
	expression(Exp, Ctx, Doc),
	findall(Step,
		(	query_compiler:set_if_var(Number, Doc, Ctx, Step)
		;	query_compiler:match_equals(Number0, Doc, Step)
		),
		Pipeline).

%% compare two expressions
comparison(Exp, _Ctx, []) :-
	ground(Exp),!,
	call(Exp).

comparison(Exp, Ctx, [
		['$set',   ['t_success', [MngOperator, array([Doc1,Doc2])]]],
		['$match', ['t_success', bool(true)]],
		['$unset', string('t_success')]
	]) :-
	Exp =.. [PlOperator, Left, Right],
	expression_operator(PlOperator, MngOperator),
	expression(Left, Ctx, Doc1),
	expression(Right, Ctx, Doc2).

%% variables
expression(Var, Ctx, string(VarValue)) :-
	var(Var),!,
	query_compiler:var_key(Var,Ctx,Key),
	atom_concat('$',Key,VarValue).
%% functions
expression(Exp, Ctx, Doc) :-
	compound(Exp),!,
	Exp =.. [Functor|Args],
	expression_function(Functor, Operator),
	findall(Out,
		(	member(In,Args),
			expression(In,Ctx,Out)
		),
		SubDocs
	),
	(	SubDocs=[Single]
	->	Doc=[Operator, Single]
	;	Doc=[Operator, array(SubDocs)]
	).
%% constant values
expression(Val, _Ctx, double(Val)) :- number(Val),!.
%% named constants
expression(pi,      _Ctx, double(V))          :- V is pi,!.
expression(e,       _Ctx, double(V))          :- V is e,!.
expression(epsilon, _Ctx, double(V))          :- V is epsilon,!.
expression(inf,     _Ctx, double('Infinity')) :- !.
expression(nan,     _Ctx, double('NaN'))      :- !.

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

test('is(-Y,+X)'):-
	lang_query:test_command(
		(Y is X), X, -3.25),
	assert_equals(Y, -3.25).

test('is(-Y,+Exp)'):-
	lang_query:test_command(
		(Y is (X + 0.5)*2), X, 2.5),
	assert_equals(Y, 6.0).

test('is(+Y,+X)') :-
	assert_true(lang_query:test_command(
		(3.0 is X), X, 3)),
	assert_false(lang_query:test_command(
		(4.0 is X), X, 3)).

test('<(+X,+Y)'):-
	assert_true(lang_query:test_command(
		(X < 7.0), X, 6)),
	assert_false(lang_query:test_command(
		(X < 7.0), X, 8)),
	assert_false(lang_query:test_command(
		(X < 7.0), X, 7)).

test('<(+Exp1,+Exp2)'):-
	assert_true(lang_query:test_command(
		((X*2) < (X + 5)), X, 2)),
	assert_false(lang_query:test_command(
		((X*2) < (X + 5)), X, 5)),
	assert_false(lang_query:test_command(
		((X*2) < (X + 5)), X, 6)).

test('between(+Low,+High,-Value)'):-
	findall(Value,
		lang_query:test_command(
			between(Low,10,Value), Low, 8),
		Values),
	assert_equals(Values, [8,9,10]).

:- end_tests('lang_arithmetic').
