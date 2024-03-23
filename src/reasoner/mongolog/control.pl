:- module(mongolog_control, []).
/** <module> Control structures in mongolog programs.

The following predicates are supported:

| Predicate   | Arguments |
| ---         | ---       |
| fail/0      | |
| false/0     | |
| true/0      | |
| !/0         | |
| \+/1        | :Goal |
| ->/2        | :Condition, :Action |
| ;/2         | :Goal1, :Goal2 |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=control
@license BSD
*/

:- use_module('mongolog').

%% query commands
:- mongolog:add_command(fail).
:- mongolog:add_command(false).
:- mongolog:add_command(true).
:- mongolog:add_command(!).
:- mongolog:add_command(\+).
:- mongolog:add_command(->).
%:- mongolog:add_command(*->).
:- mongolog:add_command(;).

%% false
%
% Same as fail, but the name has a more declarative connotation.
%
mongolog:step_expand(false, fail).

%% not(:Goal):
%
% True if Goal cannot be proven.
% Retained for compatibility only. New code should use \+/1.
%
mongolog:step_expand(not(Goal), Expanded) :-
	mongolog:step_expand(\+(Goal), Expanded).

%% \+ :Goal:
%
% True if‘Goal' cannot be proven (mnemonic: + refers to provable and
% the backslash (\) is normally used to indicate negation in Prolog).
%
mongolog:step_expand(\+(Goal), Expanded) :-
	mongolog_expand(Goal, GoalExpanded),
	% another way to write it:
	%Expanded=((call(GoalExpanded),!,fail) ; true),
	Expanded = (
		findall([], once(GoalExpanded), L),
		length(L,0)
	).

%% :Condition -> :Action
% If-then and If-Then-Else.
% The ->/2 construct commits to the choices made at its left-hand side,
% destroying choice points created inside the clause (by ;/2),
% or by goals called by this clause. Unlike !/0,
% the choice point of the predicate as a whole (due to multiple clauses)
% is not destroyed.
%
% Please note that (If -> Then) acts as (If -> Then ; fail),
% making the construct fail if the condition fails.
% This unusual semantics is part of the ISO and all de-facto Prolog standards. 
%
mongolog:step_expand(';'('->'(If,Then),Else), ';'(X,Y)) :-
	% (If -> Then) ; Else -> (If, !, Then) ; Else
	mongolog_expand([If, !, Then], X),
	mongolog_expand(Else,          Y).

mongolog:step_expand('->'(If,Then), Epanded) :-
	% (If -> Then) -> (If -> Then ; fail)
	mongolog:step_expand(';'('->'(If,Then),fail), Epanded).

%% :Goal1 ; :Goal2
% Make sure goals of disjunction are expanded.
%
mongolog:step_expand(';'(A0,A1), ';'(B0,B1)) :-
	mongolog_expand(A0,B0),
	mongolog_expand(A1,B1).

%% true
%
% Always succeed. 
%
mongolog:step_compile(true,  _, [], []).

%% fail
%
% Always fail. 
%
mongolog:step_compile(fail,  _, [['$match', ['$expr', bool(false)]]], []).

%% !
% Cut. Discard all choice points created since entering the predicate in which
% the cut appears. In other words, commit to the clause in which the cut appears
% and discard choice points that have been created by goals to the left of the cut
% in the current clause. Meta calling is opaque to the cut. This implies that cuts that
% appear in a term that is subject to meta-calling (call/1) only affect choice points
% created by the meta-called term.
%
% To realize cut, every clause [X0,...,Xn,!|_] is rewritten as [limit(1, [X0,....,Xn])|_]
% NOTE: the rewriting is currently a special case in compiler.pl and cannot be handled
%         through existing interfaces for commands in this file.
% NOTE: but disjunction below handles cut in disjunction goals
%
%mongolog:step_compile('!', _, [['$limit',int(1)]]).

%% :Goal1 ; :Goal2
% The ‘or' predicate.
% Unfortunately mongo does not support disjunction of aggregate pipelines.
% So we proceed as follows:
% 1. for each goal, use $lookup to store results into some array field
% 2. then concat all these array fields into single array stored in field 'next'
% 3. unwind the next array
%
mongolog:step_compile(';'(A,B), Context, Pipeline, StepVars) :-
	% get disjunction as list
	semicolon_list(';'(A,B), Goals),
	% generate one variable for each goal
	length(Goals,NumGoals),
	length(FindallVars,NumGoals),
	% get a list of tuples (pipeline, list variable key)
	% the result of each pipeline is written to a list,
	% and resulting lists are concatenated later to
	% achieve disjunction.
	compile_disjunction(Goals, FindallVars, [], Context, FindallStages, StepVars0),
	FindallStages \== [],
	aggregate_disjunction(FindallStages, StepVars0, Pipeline, StepVars).

%%
%aggregate_disjunction([[_,_,SingleGoal]], StepVars, Pipeline, StepVars) :-
%	% special handling in case the disjunction compiles into a single goal
%	% no disjunction needed then.
%	!,
%	mongolog:compile_term(SingleGoal, Pipeline, OuterVars->_InnerVars, StepVars, Context).

aggregate_disjunction(FindallStages, StepVars, Pipeline, StepVars) :-
	% get a list of list variable keys
	findall(string(X),
		member([_,X,_],FindallStages),
		VarKeys),
	% prepend "$" for accessing values
	maplist([string(In),string(Out)]>>
		atom_concat('$',In,Out),
		VarKeys, VarValues),
	%
	findall(QueryStage,
		% first, compute array of results for each facet
		(	member([QueryStage,_,_], FindallStages)
		% second, concatenate the results
		;	QueryStage=['$set', ['next', ['$concatArrays', array(VarValues)]]]
		% third, delete unneeded array
		;	QueryStage=['$unset', array(VarKeys)]
		% unwind all solutions from disjunction
		;	QueryStage=['$unwind', string('$next')]
		% finally project the result of a disjunction goal
		;	mongolog:set_next_vars(StepVars, QueryStage)
		% and unset the next field
		;	QueryStage=['$unset', string('next')]
		),
		Pipeline
	).

%%
% each goal in a disjunction compiles into a lookup expression.
%
compile_disjunction([], [], _, _, [], []) :- !.

compile_disjunction(
		[Goal|RestGoals],
		[Var|RestVars],
		CutVars, Ctx,
		[[QueryStage,Key,Goal]|Ys],
		StepVars) :-
	option(outer_vars(OuterVarsOrig), Ctx),
	option(global_vars(GlobalVarsOrig), Ctx, []),
	option(orig_vars(_CopiedVars0), Ctx, []),
	option(copy_vars(CopiedVars1), Ctx, []),
	option(disj_vars(DisjVars), Ctx, []),
	% ensure goal is a list
	(	is_list(Goal) -> Goal0=Goal
	;	comma_list(Goal, Goal0)
	),
	% compile-time grounding of variables can be done in goals of a disjunction.
	% however, a variable referred to in different goals of a disjunction cannot
	% be instantiated to the same value compile-time.
	% the workaround is to create a copy of the goal here,
	% and make sure the different variables are accessed in mongo with a common key.
	copy_term(Goal0, GoalCopy),
	term_variables(Goal0,    VarsOrig),
	term_variables(GoalCopy, VarsCopy),
	%
	copy_vars(OuterVarsOrig,  VarsOrig, VarsCopy, OuterVarsCopy),
	copy_vars(GlobalVarsOrig, VarsOrig, VarsCopy, GlobalVarsCopy),
	copy_vars(DisjVars,       VarsOrig, VarsCopy, DisjVarsCopy),
	% remember the mapping between original and copy of the variables,
	% This is important as the copies may receive groundings in the compilation
	% process (when lookup_array is called)
	pairs_keys_values(VV, VarsOrig, VarsCopy),
	copy_vars(CopiedVars1, VarsOrig, VarsCopy, CopiedVars2),
	%
	get_varkeys(Ctx, CopiedVars2, VV, VOs, VCs),
	% add match command checking for all previous goals with cut having
	% no solutions (size=0)
	findall([CutVarKey, ['$size', int(0)]],
		member([CutVarKey, _],CutVars),
		CutMatches0),
	(	CutMatches0=[] -> CutMatches=[]
	;	CutMatches=[['$match', CutMatches0]]
	),
	% since step_var does not list CutVars, we need to add them here to context
	% such that they will be accessible in lookup
	append(OuterVarsCopy, CutVars, OuterVarsCopy0),
	merge_options([
		outer_vars(OuterVarsCopy0),
		global_vars(GlobalVarsCopy),
		disj_vars(DisjVarsCopy),
		orig_vars(VOs),
		copy_vars(VCs),
		additional_vars(CutVars)
	], Ctx, InnerCtx),
	% compile the step
	mongolog:var_key(Var, Ctx, Key),
	mongolog:lookup_array(Key, GoalCopy, CutMatches, [],
		InnerCtx, StepVars_copy, QueryStage),
	!,
	% check if this goal has a cut, if so extend CutVars list
	(	has_cut(Goal) -> CutVars0=[[Key,Var]|CutVars]
	;	CutVars0=CutVars
	),
	copy_vars(StepVars_copy, VarsCopy, VarsOrig, StepVars_this),
	append(StepVars_this, DisjVars, DisjVars0),
	list_to_set(DisjVars0, DisjVars1),
	% compile remaining goals
	merge_options([
		outer_vars(OuterVarsOrig),
		disj_vars(DisjVars1)
	], Ctx, RestCtx),
	compile_disjunction(RestGoals, RestVars, CutVars0,
		RestCtx, Ys, StepVars_rest),
	%
	%resolve_vars(StepVars_copy, OuterVarsOrig0, StepVars_this),
	append(StepVars_this, StepVars_rest, StepVars0),
	list_to_set(StepVars0, StepVars).

compile_disjunction([_|Goals], [_|Vars], CutVars, Ctx, Pipelines, StepVars) :-
	% skip goal if compilation was "surpressed" above
	compile_disjunction(Goals, Vars, CutVars, Ctx, Pipelines, StepVars).

%%
% Create a copy of a variable map with fresh variables in the copy
% but the same keys.
%
copy_vars(ReferredVars, TermVarsOrig, TermVarsCopy, CopiedVars) :-
	copy_vars1(ReferredVars, TermVarsOrig, TermVarsCopy, CopiedVars0),
	copy_vars2(ReferredVars, CopiedVars0, CopiedVars).
	
copy_vars1(_, [], [], []) :- !.
copy_vars1(ReferredVars, [X|Xs], [Y|Ys], [[Key,Y]|Zs]) :-
	member([Key,Z], ReferredVars),
	Z == X,
	!,
	copy_vars1(ReferredVars, Xs, Ys, Zs).
copy_vars1(ReferredVars, [_|Xs], [_|Ys], Zs) :-
	copy_vars1(ReferredVars, Xs, Ys, Zs).

%%
copy_vars2([], _, []) :- !.
copy_vars2([[Key,X]|Xs], Vars1, [[Key,term(Z)]|Ys]) :-
	compound(X),
	X=term(X1),!,
	copy_vars2([[Key,X1]|Xs], Vars1, [[Key,Z]|Ys]).
copy_vars2([[Key,X]|Xs], Vars1, [[Key,Z]|Ys]) :-
	(	memberchk([Key,Y],Vars1)
	->	Z=Y
	;	Z=X
	),
	copy_vars2(Xs,Vars1,Ys).

%
get_varkeys(_, _, [], [], []) :- !.
get_varkeys(Ctx, ParentVars,
		[VO-VC|VV],
		[[Key,VO]|VOs],
		[[Key,VC]|VCs]) :-
	once((
		( member([Key,X],ParentVars), X == VO )
	;	( mongolog:var_key(VO, Ctx, Key) )
	)),
	get_varkeys(Ctx,ParentVars,VV,VOs,VCs).

%%
has_cut('!') :- !.
has_cut(Goal) :-
	is_list(Goal), !,
	% FIXME: cut was replaced with limit before. seems at the moment we cannot distinguish cut from regular limit(1) here :/
	memberchk(limit(1,_),Goal).
has_cut(Goal) :-
	comma_list(Goal,List),
	has_cut(List).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_control').

test('(+Goal ; +Goal)'):-
	findall(X,
		mongolog:test_call(
			(	(X is (Num + 5))
			;	(X is (Num * 2))
			),
			Num, 4.5),
		Results),
	assert_unifies(Results,[_,_]),
	assert_true(ground(Results)),
	assert_true(memberchk(9.5,Results)),
	assert_true(memberchk(9.0,Results)).

test('(+Goal ; fail)'):-
	findall(X,
		mongolog:test_call(
			(	(X is (Num + 5))
			;	fail
			),
			Num, 4.5),
		Results),
	assert_equals(Results,[9.5]).

test('(fail ; fail)'):-
	assert_false(mongolog:test_call(
		((Num > 5) ; fail), Num, 4.5)).

test('(+Goal ; true)'):-
	findall(X,
		mongolog:test_call(
			(	(X is (Num + 5))
			;	true
			),
			Num, 4.5),
		Results),
	assert_unifies(Results,[_,_]),
	assert_true(memberchk(9.5,Results)),
	assert_true(once((member(Var,Results), var(Var)))).

test('(+Goal ; $early_evaluated)'):-
	% `X is 15` is evaluated compile-time, while
	% the other term must be computed at run-time. 
	findall(X,
		mongolog:test_call(
			(	(X is (Num + 5))
			;	(X is 15)
			),
			Num, 4.5),
		Results),
	assert_unifies(Results,[_,_]),
	assert_true(memberchk(9.5,Results)),
	assert_true(memberchk(15.0,Results)).

test('(+Goal ; +PrunedGoal)'):-
	mongolog:test_call(
		(	(X is (Num + 5))
		;	(7 < 5, X is (Num * 2))
		),
		Num, 4.5),
	assert_equals(X,9.5).

test('((+Goal ; +Goal), !)'):-
	mongolog:test_call(
		(	(	(X is (Num + 5))
			;	(X is (Num * 2))
			), !
		),
		Num, 4.5),
	assert_equals(X,9.5).

test('(((+Goal,+Goal) ; (+Goal,+Goal)))') :-
	findall([X,Y],
		mongolog:test_call(
			(	(X is Num, Y is X + 1)
			;	(X is Num, Y is X + 2)
			),
			Num, 4.5),
		Results),
	assert_equals(Results, [[4.5,5.5], [4.5,6.5]]).

test('(((+G ; +G), +G) ; +G)') :-
	findall([X,Y],
		mongolog:test_call(
			(	(X is Num, ((X < 2.0 ; X > 4.0), Y is X + 1))
			;	(X is Num, Y is X + 2)
			),
			Num, 4.5),
		Results),
	assert_equals(Results, [[4.5,5.5], [4.5,6.5]]).

test('((+If -> +Then) ; +Else)::Then') :-
	mongolog:test_call(
		(	Num > 5 -> X is Num * 2
		;	X is Num + 2
		),
		Num, 5.5),
	assert_equals(X,11.0).

test('((+If -> +Then) ; +Else)::Else'):-
	mongolog:test_call(
		(	Num > 5 -> X is Num * 2
		;	X is Num + 2
		),
		Num, 4.5),
	assert_equals(X,6.5).

test('\\+(+Goal)'):-
	assert_true(mongolog:test_call(
		\+(Number > 5), Number, 4.5)),
	assert_false(mongolog:test_call(
		\+(Number > 4), Number, 4.5)).

:- end_tests('mongolog_control').
