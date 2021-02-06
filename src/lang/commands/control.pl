:- module(control_commands, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(true,  [ask,tell]).
:- query_compiler:add_command(false, [ask,tell]).
:- query_compiler:add_command(fail,  [ask,tell]).
:- query_compiler:add_command(\+,    [ask]).
:- query_compiler:add_command(!,     [ask,tell]).
:- query_compiler:add_command(->,    [ask]).
%:- query_compiler:add_command(*->, [ask]).
:- query_compiler:add_command(;,     [ask]).

%% false
%
% Same as fail, but the name has a more declarative connotation.
%
query_compiler:step_expand(false, fail, _Mode).

%% not(:Goal):
%
% True if Goal cannot be proven.
% Retained for compatibility only. New code should use \+/1.
%
query_compiler:step_expand(not(Goal), Expanded, Mode) :-
	query_compiler:step_expand(\+(Goal), Expanded, Mode).

%% \+ :Goal:
%
% True if‘Goal' cannot be proven (mnemonic: + refers to provable and
% the backslash (\) is normally used to indicate negation in Prolog).
%
query_compiler:step_expand(\+(Goal), Expanded, Mode) :-
	% another way to write it:
	%Rewritten=((call(Goal),!,fail) ; true),
	Rewritten=(findall([], once(Goal), L), length(L,0)),
	query_expand(Rewritten, Expanded, Mode).

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
query_compiler:step_expand(';'('->'(If,Then),Else), ';'(X,Y), Mode) :-
	% (If -> Then) ; Else -> (If, !, Then) ; (!, Else)
	query_expand([If, !, Then], X, Mode),
	query_expand([!, Else],     Y, Mode).

query_compiler:step_expand('->'(If,Then), Epanded, Mode) :-
	% (If -> Then) -> (If -> Then ; fail)
	query_compiler:step_expand(';'('->'(If,Then),fail), Epanded, Mode).

%% TODO: :Condition *-> :Action ; :Else
% This construct implements the so-called‘soft-cut'.
% The control is defined as follows: If Condition succeeds at least once,
% the semantics is the same as (call(Condition), Action).
% If Condition does not succeed, the semantics is that of (\+ Condition, Else).
% In other words, if Condition succeeds at least once, simply behave as the
% conjunction of call(Condition) and Action, otherwise execute Else.
% The construct is known under the name if/3 in some other Prolog implementations. 
%
%query_compiler:step_expand(
%		';'('*->'(Condition,Action),Else),
%		';'(X,Y), Mode) :-
%	fail.
	
%% :Goal1 ; :Goal2
% Make sure goals of disjunction are expanded.
%
query_compiler:step_expand(';'(A0,A1), ';'(B0,B1), Context) :-
	query_expand(A0,A1,Context),
	query_expand(B0,B1,Context).

%% true
%
% Always succeed. 
%
query_compiler:step_compile(true,  _, []).

%% fail
%
% Always fail. 
%
query_compiler:step_compile(fail,  _, [['$match', ['$expr', bool(false)]]]).

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
% NOTE: but disjunction below handles cut in disunction goals
%
%query_compiler:step_compile('!', _, [['$limit',int(1)]]).

%% :Goal1 ; :Goal2
% The ‘or' predicate.
% Unfortunately mongo does not support disjunction of aggregate pipelines.
% So we proceed as follows:
% 1. for each goal, use $lookup to store results into some array field
% 2. then concat all these array fields into single array stored in field 'next'
% 3. unwind the next array
%
query_compiler:step_compile(';'(A,B), Context, Pipeline) :-
	% get disjunction as list
	semicolon_list(';'(A,B), Goals),
	% read options from context
	% option(mode(ask), Context),
	option(step_vars(StepVars), Context),
	option(outer_vars(OuterVars), Context),
	% generate one variable for each goal
	length(Goals,NumGoals),
	length(FindallVars,NumGoals),
	% get a list of tuples (pipeline, list variable key)
	% the result of each pipeline is written to a list,
	% and resulting lists are concatenated later to
	% achieve disjunction.
	compile_disjunction(Goals, FindallVars, [], Context, FindallStages),
	FindallStages \= [],
	% special handling in case the disjunction compiles into a single goal
	% no disjunction needed then.
	(	FindallStages=[[_,_,SingleGoal]]
	->	query_compiler:compile_term(SingleGoal, Pipeline, OuterVars->_, Context)
	;	aggregate_disjunction(FindallStages, StepVars, Pipeline)
	).

%%
aggregate_disjunction(FindallStages, StepVars, Pipeline) :-
	% get a list of list variable keys
	findall(string(X),
		member([_,X,_],FindallStages),
		VarKeys),
	% prepend "$" for accessing values
	maplist([string(In),string(Out)]>>
		atom_concat('$',In,Out),
		VarKeys, VarValues),
	%
	findall(Stage,
		% first, compute array of results for each facet
		(	member([Stage,_,_], FindallStages)
		% second, concatenate the results
		;	Stage=['$set', ['next', ['$concatArrays', array(VarValues)]]]
		% third, delete unneeded array
		;	Stage=['$unset', array(VarKeys)]
		% unwind all solutions from disjunction
		;	Stage=['$unwind', string('$next')]
		% finally project the result of a disjunction goal
		;	query_compiler:set_next_vars(StepVars, Stage)
		% and unset the next field
		;	Stage=['$unset', string('next')]
		),
		Pipeline
	).

%%
% each goal in a disjunction compiles into a lookup expression.
%
compile_disjunction([], [], _, _, []) :- !.

compile_disjunction(
		[Goal|RestGoals],
		[Var|RestVars],
		CutVars, Ctx,
		[[Stage,Key,Goal]|Ys]) :-
	% ensure goal is a list
	(	is_list(Goal) -> Goal0=Goal
	;	comma_list(Goal, Goal0)
	),
	% compile-time grounding of variables can be done in goals of a disjunction.
	% however, a variable referred to in different goals of a disjunction cannot
	% be instantiated to the same value compile-time.
	% the workaround is to create a copy of the goal here,
	% and make sure the different variables are accessed in mongo with a common key.
	% Then, the unification is outcarried later, after mongo has returned a match.
	copy_term(Goal0, GoalCopy),
	term_variables(Goal0,    VarsOrig),
	term_variables(GoalCopy, VarsCopy),
	copy_context(Ctx, VarsOrig, VarsCopy, CtxCopy),
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
	select(outer_vars(OuterVars), CtxCopy, InnerCtx),
	append(OuterVars, CutVars, OuterVars0),
	% compile the step
	query_compiler:var_key(Var, Ctx, Key),
	query_compiler:lookup_array(Key, GoalCopy, CutMatches, [],
		[outer_vars(OuterVars0)|InnerCtx], _,
		Stage),
	!,
	% check if this goal has a cut, if so extend CutVars list
	(	has_cut(Goal) -> CutVars0=[[Key,Var]|CutVars]
	;	CutVars0=CutVars
	),
	% continue with rest
	compile_disjunction(RestGoals, RestVars,
		CutVars0, Ctx, Ys).

compile_disjunction([_|Goals], [_|Vars], CutVars, Ctx, Pipelines) :-
	% skip goal if compilation was "surpressed" above
	compile_disjunction(Goals, Vars, CutVars, Ctx, Pipelines).

%%
copy_context(Orig, VarsOrig, VarsCopy,
		[ outer_vars(OuterVarsCopy),
		  step_vars(StepVarsCopy) | Buf1 ]) :-
	select(outer_vars(OuterVars), Orig, Buf0),
	select(step_vars(StepVars), Buf0, Buf1),
	copy_context_vars(OuterVars, VarsOrig, VarsCopy, OuterVarsCopy),
	copy_context_vars(StepVars, VarsOrig, VarsCopy, StepVarsCopy).

copy_context_vars([], _, _, []) :- !.
copy_context_vars([X|Xs], VarsOrig, VarsCopy, [Y|Ys]) :-
	once((
		copy_context_var(X, VarsOrig, VarsCopy, Y)
	;	Y=X
	)),
	copy_context_vars(Xs, VarsOrig, VarsCopy, Ys).

copy_context_var([Key,VarOrig], VarsOrig, VarsCopy, [Key,VarCopy]) :-
	% NOTE: nth0/3 unifies unbound variables, so we _cannot_ call:
	%        `nth0(Index, VarsOrig, VarOrig)`
	% TODO: is there a variant of nth0 using equality check instead of unification?
	%          or map to varkeys before in VarsOrig
	nth0(Index, VarsOrig, X),
	X == VarOrig,
	nth0(Index, VarsCopy, VarCopy),
	!.

%%
has_cut('!') :- !.
has_cut(Goal) :-
	is_list(Goal), !,
	% FIXME: cut was replaced with limit before. seems at the moment
	%        we cannot distinguish cut from regular limit(1) here :/
	memberchk(limit(1,_),Goal).
has_cut(Goal) :-
	comma_list(Goal,List),
	has_cut(List).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('control_commands').

test('(+Goal ; +Goal)'):-
	findall(X,
		lang_query:test_command(
			(	(X is (Num + 5))
			;	(X is (Num * 2))
			),
			Num, 4.5),
		Results),
	assert_unifies(Results,[_,_]),
	assert_true(memberchk(9.5,Results)),
	assert_true(memberchk(9.0,Results)).

test('(+Goal ; fail)'):-
	findall(X,
		lang_query:test_command(
			(	(X is (Num + 5))
			;	fail
			),
			Num, 4.5),
		Results),
	assert_equals(Results,[9.5]).

test('(+Goal ; true)'):-
	findall(X,
		lang_query:test_command(
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
		lang_query:test_command(
			(	(X is (Num + 5))
			;	(X is 15)
			),
			Num, 4.5),
		Results),
	assert_unifies(Results,[_,_]),
	assert_true(memberchk(9.5,Results)),
	assert_true(memberchk(15.0,Results)).

test('(+Goal ; +PrunedGoal)'):-
	lang_query:test_command(
		(	(X is (Num + 5))
		;	(7 < 5, X is (Num * 2))
		),
		Num, 4.5),
	assert_equals(X,9.5).

test('((+Goal ; +Goal), !)'):-
	lang_query:test_command(
		(	(	(X is (Num + 5))
			;	(X is (Num * 2))
			), !
		),
		Num, 4.5),
	assert_equals(X,9.5).

test('(((+Goal,+Goal) ; (+Goal,+Goal)))') :-
	findall([X,Y],
		lang_query:test_command(
			(	(X is Num, Y is X + 1)
			;	(X is Num, Y is X + 2)
			),
			Num, 4.5),
		Results),
	assert_equals(Results, [[4.5,5.5], [4.5,6.5]]).

test('((+If -> +Then) ; +Else)::Then') :-
	lang_query:test_command(
		(	Num > 5 -> X is Num * 2
		;	X is Num + 2
		),
		Num, 5.5),
	assert_equals(X,11.0).

test('((+If -> +Then) ; +Else)::Else'):-
	lang_query:test_command(
		(	Num > 5 -> X is Num * 2
		;	X is Num + 2
		),
		Num, 4.5),
	assert_equals(X,6.5).

test('\\+(+Goal)'):-
	assert_true(lang_query:test_command(
		\+(Number > 5), Number, 4.5)),
	assert_false(lang_query:test_command(
		\+(Number > 4), Number, 4.5)).

:- end_tests('control_commands').
