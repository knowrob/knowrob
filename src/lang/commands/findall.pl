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
query_compiler:step_var(
		findall(Template, _, List), Ctx,
		[List_var, list(List,Template)]) :-
	query_compiler:var_key(List, Ctx, List_var).

%% findall(+Template, :Goal, -Bag)
% Create a list of the instantiations Template gets successively on
% backtracking over Goal and unify the result with Bag.
% Succeeds with an empty list if Goal has no solutions.
%
query_compiler:step_compile(
		findall(Template, Terminals, List),
		Ctx, Pipeline) :-
	% option(mode(ask), Context),
	findall(Step,
		% perform lookup, collect results in 'next' array
		(	query_compiler:lookup_array('next',Terminals,
				[], [], Ctx, _, Step)
		% $set the list variable field from 'next' field
		;	set_result(Template, List, Ctx, Step)
		% array at 'next' field not needed anymore
		;	Step=['$unset', string('next')]
		),
		Pipeline).

%%
% findall $set receives a list of matching documents in "next" field.
% $set uses additional $map operation to only keep the fields of
% variables referred to in Template.
%
set_result(Template, List, Ctx,
	['$set',
		[List_Key, ['$map',[
			['input',string('$next')],
			['in', ElemProjection]
		]]]
	]) :-
	query_compiler:var_key(List, Ctx, List_Key),
	term_variables(Template, PatternVars),
	%
	findall([Key, string(Val)],
		(	(	Key='v_scope', Val='$$this.v_scope' )
		;	(	member(Var,PatternVars),
				query_compiler:var_key(Var, Ctx, Key),
				atom_concat('$$this.', Key, Val)
			)
		),
		ElemProjection).

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
	assert_true(((Results=[Var|_],var(Var)))).

test('findall with list pattern', [fixme('findall with pattern not working')]):-
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

test('findall with term pattern', [fixme('findall with pattern not working')]):-
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
		test(9.5,10.5),
		test(9.0,11.0)
	]).

:- end_tests('lang_findall').
