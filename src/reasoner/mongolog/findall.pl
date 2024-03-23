:- module(mongolog_findall, []).
/** <module> Finding all solutions to a goal in mongolog programs.

The following predicates are supported:

| Predicate    | Arguments |
| ---          | ---       |
| findall/3    | +Template, :Goal, -Bag |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=allsolutions
@license BSD
*/

:- use_module('mongolog').

%% register query commands
:- mongolog:add_command(findall).

%%
mongolog:step_expand(
		findall(Template, Goal, List),
		findall(Template, Expanded, List)) :-
	mongolog_expand(Goal, Expanded).

%% findall(+Template, :Goal, -Bag)
% Create a list of the instantiations Template gets successively on
% backtracking over Goal and unify the result with Bag.
% Succeeds with an empty list if Goal has no solutions.
%
mongolog:step_compile(
		findall(Template, Terminals, List),
		Ctx, Pipeline, StepVars) :-
	% findall only exposes the List variable to the outside.
	mongolog:step_vars(List, Ctx, StepVars),
	mongolog:var_key_or_val(List, Ctx, List0),
	% add template vars to compile context.
	% this is important to enforce that vars in Template are referred
	% to with a common key within findall.
	% note that the vars should not be added to the "outer_vars"
	% array as variables in template are _not_ exposed to the outside.
	mongolog:step_vars(Template, Ctx, TemplateVars),
	% TODO: redundant, make an interface in query compiler
	once((select(disj_vars(DisjVars), Ctx, Ctx0);(DisjVars=[],Ctx0=Ctx))),
	append(DisjVars, TemplateVars, DisjVars0),
	list_to_set(DisjVars0,DisjVars1),
	Ctx1=[disj_vars(DisjVars1)|Ctx0],
	% Get the $map expression to instantiate the template for each list element.
	% NOTE: it is not allowed due to handling here to construct
	% the pattern in a query, it must be given in the findall command compile-time.
	% if really needed it could be done more dynamic, I think.
	template_instantiation(Template, Ctx1, Instantiation),
	findall(Step,
		% perform lookup, collect results in 'next' array
		(	mongolog:lookup_array('t_next',Terminals,
				[], [], Ctx1, _, Step)
		% $set the list variable field from 'next' field
		;	Step=['$set', ['t_list', ['$map',[
					['input',string('$t_next')],
					['in', Instantiation] ]]]]
		;	mongolog:set_if_var(List, string('$t_list'), Ctx1, Step)
		;	mongolog:match_equals(List0, string('$t_list'), Step)
		% array at 'next' field not needed anymore
		;	Step=['$unset', array([string('t_next'), string('t_list')])]
		),
		Pipeline).

%%
% findall template must be given compile-time to construct the mongo expression
% to map lookup results to be a proper instantiation of the template.
% the currently mapped array element is referred to as "$$this"
%
template_instantiation(Var, Ctx, string(Val)) :-
	% for variables in template, lookup in compile context
	% or create a key used in mongo to refer to the var
	mongolog:var_key(Var, Ctx, Key),
	atom_concat('$$this.', Key, Val).

template_instantiation(List, Ctx, array(Elems)) :-
	is_list(List),!,
	findall(Y,
		(	member(X,List),
			template_instantiation(X, Ctx, Y)
		),
		Elems).

template_instantiation(Template, Ctx, [
		['type', string('compound')],
		['value', [
			['functor', string(Functor)],
			['args', Args0]
		]]
	]) :-
	compound(Template),!,
	Template =.. [Functor|Args],
	template_instantiation(Args, Ctx, Args0).
	
template_instantiation(Atomic, _Ctx, Constant) :-
	atomic(Atomic),
	mongolog:get_constant(Atomic, Constant).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_findall').

test('findall(+Succeeds)'):-
	mongolog:test_call(
		findall(X,
			X is (Num + 5),
			Results),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_equals(Results,[9.5]).

test('findall(+Succeeds ; +Succeeds)'):-
	mongolog:test_call(
		(	findall(X,
				(	(X is (Num + 5))
				;	(X is (Num * 2))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_equals(Results,[9.5,9.0]).

test('findall(+,:,+)'):-
	assert_true(mongolog:test_call(
		(	findall(X,
				(	(X is (Num + 5))
				;	(X is (Num * 2))
				),
				[9.5,9.0])
		),
		Num, double(4.5))).

test('findall(+Succeeds ; +Fails)'):-
	mongolog:test_call(
		(	findall(X,
				(	(X is (Num + 5))
				;	(Num > 5, X is (Num * 2))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_equals(Results,[9.5]).

test('findall(+Succeeds ; +Fails ; +Succeeds)'):-
	mongolog:test_call(
		(	findall(X,
				(	(X is (Num + 5))
				;	(Num > 5, X is (Num * 2))
				;	(X is (Num + 6))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_equals(Results,[9.5,10.5]).

test('findall with ungrounded'):-
	mongolog:test_call(
		(	findall(X,
				(	true
				;	(X is (Num * 2))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_unifies(Results,[_,9.0]),
	( Results=[Var|_] -> assert_true(var(Var)) ; true ).

test('findall 1-element list'):-
	mongolog:test_call(
		(	findall([X],
				(	X is Num + 5
				;	X is Num * 2
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_unifies(Results,[[9.5],[9.0]]).

test('findall 2-element list'):-
	mongolog:test_call(
		(	findall([X,Y],
				(	(X is (Num + 5), Y is X + 1)
				;	(X is (Num * 2), Y is X + 2)
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_unifies(Results,[[9.5,10.5],[9.0,11.0]]).

test('findall 1-ary term'):-
	mongolog:test_call(
		(	findall(test(X),
				(	X is (Num + 5)
				;	X is (Num * 2)
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_unifies(Results,[test(9.5), test(9.0)]).

test('findall 2-ary term'):-
	mongolog:test_call(
		(	findall(test(X,Y),
				(	(X is (Num + 5), Y is X + 1)
				;	(X is (Num * 2), Y is X + 2)
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_true(ground(Results)),
	assert_unifies(Results,[
		test(9.5,10.5), test(9.0,11.0) ]).

:- end_tests('mongolog_findall').
