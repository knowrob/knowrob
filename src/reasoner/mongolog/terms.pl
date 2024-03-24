:- module(mongolog_terms, []).
/** <module> Analysing and constructing terms in mongolog programs.

The following predicates are supported:

| Predicate    | Arguments |
| ---          | ---       |
| functor/3    | ?Term, ?Name, ?Arity |
| arg/3        | ?Arg, +Term, ?Value |
| copy_term/2  | +In, -Out |
| =../2        | ?Term, ?List |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=manipterm
@license BSD
*/

:- use_module('mongolog').

%% query commands
:- mongolog:add_command(functor).
:- mongolog:add_command(arg).
:- mongolog:add_command(copy_term).
:- mongolog:add_command(=..).

%% functor(?Term, ?Name, ?Arity) [ISO]
% True when Term is a term with functor Name/Arity.
%
mongolog:step_compile(functor(Term,Functor,Arity), Ctx, Pipeline) :-
	mongolog:var_key_or_val(Term,Ctx,Term0),
	mongolog:var_key_or_val(Functor,Ctx,Functor0),
	mongolog:var_key_or_val(Arity,Ctx,Arity0),
	findall(Step,
		(	mongolog:set_if_var(Term, [
				['type', string('compound')],
				['value', [
					['functor', Functor0],
					['args', ['$map', [
						['input', ['$range', array([ integer(0), Arity0, integer(1) ])]],
						['in', [['type', string('var')], ['value', string('_')]]]
					]]]
				]]
			], Ctx, Step)
		;	Step=['$set', ['t_term', Term0]]
		;	mongolog:set_if_var(Functor,    string('$t_term.value.functor'), Ctx, Step)
		;	mongolog:match_equals(Functor0, string('$t_term.value.functor'), Step)
		;	mongolog:set_if_var(Arity,    ['$size', string('$t_term.value.args')], Ctx, Step)
		;	mongolog:match_equals(Arity0, ['$size', string('$t_term.value.args')], Step)
		;	Step=['$unset', string('t_term')]
		),
		Pipeline).

%% arg(?Arg, +Term, ?Value) [ISO]
% Term should be instantiated to a term, Arg to an integer between 1 and the arity of Term.
% Value is unified with the Arg-th argument of Term.
% Arg may also be unbound. In this case Value will be unified with the successive
% arguments of the term. On successful unification, Arg is unified with the
% argument number. Backtracking yields alternative solutions.
%
mongolog:step_compile(arg(Arg,Term,Value), Ctx, Pipeline) :-
	% FIXME: arg also need to handle var unification as in:
	%         arg(0,foo(X),Y) would imply X=Y
	%		- can be handled with conditional $set, add [X,Y] to
	%         var array if both of them are vars
	%
	mongolog:var_key_or_val(Arg,Ctx,Arg0),
	mongolog:var_key_or_val(Term,Ctx,Term0),
	mongolog:var_key_or_val(Value,Ctx,Value0),
	findall(Step,
		(	Step=['$set', ['t_term', Term0]]
		;	mongolog:set_if_var(Arg, ['$add', array([
					['$indexOfArray', array([ string('$t_term.value.args'), Value0 ])],
					integer(1)
			])], Ctx, Step)
		;	mongolog:set_if_var(Value, ['$arrayElemAt', array([
					string('$t_term.value.args'),
					['$subtract', array([Arg0, integer(1)])]	
			])], Ctx, Step)
		;	mongolog:match_equals(Value0, ['$arrayElemAt', array([
					string('$t_term.value.args'),
					['$subtract', array([Arg0, integer(1)])]	
			])], Step)
		;	Step=['$unset', string('t_term')]
		),
		Pipeline).

%% copy_term(+In, -Out) [ISO]
% Create a version of In with renamed (fresh) variables and unify it to Out.
%
mongolog:step_compile(copy_term(In,Out), Ctx, Pipeline) :-
	mongolog:var_key_or_val(In,Ctx,In0),
	mongolog:var_key(Out,Ctx,OutKey),
	findall(Step,
		(	Step=['$set', ['t_term', In0]]
		;	Step=['$set', [OutKey, ['$cond', [
				['if', ['$not', array([string('$t_term.value')])]],
				['then', string('$t_term')],
				['else', [
					['type', string('compound')],
					['value', [
						['functor', string('$t_term.value.functor')],
						['args', ['$map', [
							['input', string('$t_term.value.args')],
							['in', ['$cond', [
								% if array element is not a variable
								['if', ['$ne', array([string('$$this.type'), string('var')])]],
								% then yield the value
								['then', string('$$this')],
								% else map to new variable
								['else', [['type', string('var')], ['value', string('_')]]]
							]]]
						]]]
					]]
				]]
			]]]]
		;	Step=['$unset', string('t_term')]
		),
		Pipeline).

%% ?Term =.. ?List [ISO]
% List is a list whose head is the functor of Term and the remaining arguments
% are the arguments of the term.
% Either side of the predicate may be a variable, but not both.
% This predicate is called "Univ". 
%
mongolog:step_compile(=..(Term,List), Ctx, Pipeline) :-
	% FIXME: it won't work to unify two variables with univ yet, as in:
	%			foo(X,a) =.. [foo,Z,a] would imply X=Z which is not handled here yet!
	%          - needs additional map/filter operation
	%				- get args that are different vars in list and term, then add to var array
	%
	mongolog:var_key_or_val(Term,Ctx,Term0),
	mongolog:var_key_or_val(List,Ctx,List0),
	findall(Step,
		(	mongolog:set_if_var(Term, [
				['type', string('compound')],
				['value', [
					['functor', ['$arrayElemAt', array([List0,integer(0)])]],
					['args', ['$slice', array([
						List0, integer(1),
						['$subtract', array([['$size', List0], integer(1)])]
					])]]
				]]
			], Ctx, Step)
		;	Step=['$set', ['t_term', Term0]]
		;	mongolog:set_if_var(List, ['$concatArrays', array([
				array([string('$t_term.value.functor')]),
				string('$t_term.value.args')
			])], Ctx, Step)
		;	mongolog:match_equals(List0, ['$concatArrays', array([
				array([string('$t_term.value.functor')]),
				string('$t_term.value.args')
			])], Step)
		;	Step=['$unset', string('t_term')]
		),
		Pipeline).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_terms').

test('functor(+Term,-Functor,-Arity)'):-
	mongolog:test_call(
		functor(Term,Functor,Arity),
		Term, foo(bar,45)),
	assert_equals(Functor, foo),
	assert_equals(Arity, 2).

test('functor(-Term,+Functor,+Arity)'):-
	mongolog:test_call(
		functor(Term,foo,Arity), Arity, 2),
	assert_unifies(Term, foo(_,_)),
	assert_false(ground(Term)).

test('arg(+Index,+Term,+Value)'):-
	assert_true(mongolog:test_call(
		arg(Index,foo(a,b,c),a), Index, 1)),
	assert_true(mongolog:test_call(
		arg(Index,foo(a,b,c),b), Index, 2)),
	assert_false(mongolog:test_call(
		arg(Index,foo(a,b,c),b), Index, 1)),
	assert_false(mongolog:test_call(
		arg(Index,foo(a,b,c),d), Index, 1)).

test('arg(+Index,+Term,-Value)') :-
	mongolog:test_call(
		arg(Index,foo(a,b,c),Value), Index, 1),
	assert_equals(Value,a),
	assert_false(mongolog:test_call(
		arg(Index,foo(a,b,c),_), Index, 5)).

test('arg(-Index,+Term,+Value)'):-
	mongolog:test_call(
		arg(Index,foo(a,b,c),Value), Value, b),
	assert_equals(Index,2),
	assert_false(mongolog:test_call(
		arg(_,foo(a,b,c),Value), Value, d)).

test('arg(-UnwindedIndex,+Term,+Value)', fixme('$indexOfArray only returns the first occurence')):-
	findall(Index,
		mongolog:test_call(
			arg(Index,foo(a,b,a),Value), Value, a),
		Results),
	assert_equals(Results, [1,3]).

test('copy_term(+In,-Out)::compound') :-
	mongolog:test_call(copy_term(In,Out), In, foo(a)),
	assert_equals(Out,foo(a)).

test('copy_term(+In,-Out)::atom') :-
	mongolog:test_call(copy_term(In,Out), In, a),
	assert_equals(Out,a).

test('copy_term(+In,-Out)::number') :-
	mongolog:test_call(copy_term(In,Out), In, 7),
	assert_equals(Out,7.0).

test('copy_term(+In,-Out)::vars') :-
	mongolog:test_call(copy_term(In,Out), In, foo(a,X)),
	assert_unifies(Out, foo(a,_)),
	(Out=foo(a,Y) -> assert_true(X \== Y) ; true).

test('=..(+Term,-List)::ground') :-
	mongolog:test_call(=..(Term,List), Term, foo(a,b)),
	assert_equals(List,[foo,a,b]).

test('=..(+Term,-List)::nonground') :-
	mongolog:test_call(=..(Term,List), Term, foo(a,B)),
	assert_equals(List,[foo,a,B]).

test('=..(-Term,+List)') :-
	mongolog:test_call(=..(Term,List), List, [foo,a,b]),
	assert_equals(Term,foo(a,b)).

:- end_tests('mongolog_terms').

