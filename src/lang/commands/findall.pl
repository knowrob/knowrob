:- module(lang_findall, []).

:- use_module(library('db/mongo/client'),
		[ mng_one_db/2 ]).
:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(findall, [ask]).
% TODO: support bagof (then, setof := bagof o sort)
%:- query_compiler:add_command(bagof,   [ask]).
%:- query_compiler:add_command(setof,   [ask]).

%%
query_compiler:step_expand(
		findall(Template, Goal, List),
		findall(Template, Expanded, List),
		Context) :-
	query_expand(Goal, Expanded, Context).

%% setof(+Template, +Goal, -Set)
% Equivalent to bagof/3, but sorts the result using sort/2 to
% get a sorted list of alternatives without duplicates.
%
%query_compiler:step_expand(
%		setof(Template, Goal, Set),
%		[ bagof(Template, Expanded, List), sort(List, Set) ],
%		Context) :-
%	query_expand(Goal, Expanded, Context).

%%
% findall only exposes the List variable to the outside.
%
query_compiler:step_vars(
		findall(_, _, List), Ctx,
		[[List_var, List]]) :-
	query_compiler:var_key(List, Ctx, List_var).

%% findall(+Template, :Goal, -Bag)
% Create a list of the instantiations Template gets successively on
% backtracking over Goal and unify the result with Bag.
% Succeeds with an empty list if Goal has no solutions.
%
query_compiler:step_compile(
		findall(Template, Terminals, List),
		Ctx, Pipeline) :-
	% TODO: how does this interfere with scopes?
	%	- within findall body scopes maybe computed
	%	- member with scope should do scope intersection
	%	- for now scopes in findall are _ignored_!
	query_compiler:var_key(List, Ctx, List_Key),
	% Get the $map expression to instantiate the template for each list element.
	% NOTE: it is not allowed due to handling here to construct
	% the pattern in a query, it must be given in the findall command compile-time.
	% if really needed it could be done more dynamic, I think.
	template_instantiation(Template, Ctx, Instantiation),
	findall(Step,
		% perform lookup, collect results in 'next' array
		(	query_compiler:lookup_array('next',Terminals,
				[], [], Ctx, _, Step)
		% $set the list variable field from 'next' field
		;	Step=['$set', [List_Key, ['$map',[
					['input',string('$next')],
					['in', Instantiation] ]]]]
		% array at 'next' field not needed anymore
		;	Step=['$unset', string('next')]
		),
		Pipeline).

%%
% findall template must be given compile-time to construct the mongo expression
% to map lookup results to be a proper instantiation of the template.
% the currently mapped array element is referred to as "$$this"
%
template_instantiation(Var, Ctx, string(Val)) :-
	query_compiler:var_key(Var, Ctx, Key),
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
	query_compiler:get_constant(Atomic, Constant).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('lang_findall').

test('findall(+Succeeds ; +Succeeds)'):-
	lang_query:test_command(
		(	findall(X,
				(	(X is (Num + 5))
				;	(X is (Num * 2))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_equals(Results,[9.5,9.0]).

test('findall(+Succeeds ; +Fails)'):-
	lang_query:test_command(
		(	findall(X,
				(	(X is (Num + 5))
				;	(Num > 5, X is (Num * 2))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_equals(Results,[9.5]).

test('findall(+Succeeds ; +Fails ; +Succeeds)'):-
	lang_query:test_command(
		(	findall(X,
				(	(X is (Num + 5))
				;	(Num > 5, X is (Num * 2))
				;	(X is (Num + 6))
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_equals(Results,[9.5,10.5]).

test('findall with ungrounded'):-
	lang_query:test_command(
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
	lang_query:test_command(
		(	findall([X],
				(	X is Num + 5
				;	X is Num * 2
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_unifies(Results,[[9.5],[9.0]]).

test('findall 2-element list'):-
	lang_query:test_command(
		(	findall([X,Y],
				(	(X is (Num + 5), Y is X + 1)
				;	(X is (Num * 2), Y is X + 2)
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_unifies(Results,[[9.5,10.5],[9.0,11.0]]).

test('findall 1-ary term'):-
	lang_query:test_command(
		(	findall(test(X),
				(	X is (Num + 5)
				;	X is (Num * 2)
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_unifies(Results,[test(9.5), test(9.0)]).

test('findall 2-ary term'):-
	lang_query:test_command(
		(	findall(test(X,Y),
				(	(X is (Num + 5), Y is X + 1)
				;	(X is (Num * 2), Y is X + 2)
				),
				Results)
		),
		Num, double(4.5)
	),
	assert_unifies(Results,[
		test(9.5,10.5), test(9.0,11.0) ]).

:- end_tests('lang_findall').
