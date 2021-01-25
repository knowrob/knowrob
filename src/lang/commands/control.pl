:- module(control_commands, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(true,  [ask,tell]).
:- query_compiler:add_command(false, [ask,tell]).
:- query_compiler:add_command(fail,  [ask,tell]).
:- query_compiler:add_command('\\+', [ask]).
:- query_compiler:add_command('!',   [ask,tell]).
:- query_compiler:add_command('->',  [ask]).
%:- query_compiler:add_command('*->', [ask]).
:- query_compiler:add_command(';',   [ask]).

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
	% \+ is true iff findall yields an empty list
	query_expand(
		[ findall([], once(Goal), L), length(L,0) ],
		Expanded, Mode).

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

%%
% expose variables to the outside that appear in every facet.
% NOTE: thus it is not possible to to get the value of variables that
%       appear only in some facets.
% TODO: reconsider this
%
query_compiler:step_var(';'(A,B), Var) :-
	semicolon_list(';'(A,B), [First|Rest]),
	% make choicepoint for each variable in First
	query_compiler:step_var(First, Var),
	% only proceed if Var is a variable in each facet
	forall(
		member(Y, Rest),
		query_compiler:step_var(Y, Var)
	).

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
% To realize cut, every clause [X0,...,Xn,!|_] is rewritten as [call([X0,....,Xn,!]|_]
% NOTE: the rewriting is currently a special case in compiler.pl and cannot be handled
%         through existing interfaces for commands in this file.
%
query_compiler:step_compile('!', _, [['$limit',int(1)]]).

%% :Goal1 ; :Goal2
% The ‘or' predicate.
% Unfortunately mongo does not support disjunction of aggregate pipelines.
% So we proceed as follows:
% 1. for each facet, generate findall/3 expression and store this into some array field
% 2. then concat all these array fields into single array stored in field 'next'
% 3. unwind the next array
%
% TODO: seems like a good usecase for map-reduce?
%
query_compiler:step_compile(';'(A,B), Context, Pipeline) :-
	% get disjunction as list
	semicolon_list(';'(A,B), Goals),
	% read options from context
	% option(mode(ask), Context),
	option(step_vars(StepVars), Context),
	% get a list of tuples (pipeline, list variable key)
	% the result of each pipeline is written to a list,
	% and resulting lists are concatenated later to
	% achieve disjunction.
	% elements of the list are documents with variables
	% in StepVars being assigned.
	compile_disjunction(Goals, [], Context, FindallStages),
	% get a list of list variable keys
	findall(string(VarKey1),
		member([_,VarKey1],FindallStages),
		VarKeys),
	%
	findall(Stage,
		% first, compute array of results for each facet
		(	member([Stage,_], FindallStages)
		% second, concatenate the results
		;	Stage=['$set', ['next', ['$concatArrays', array(VarKeys)]]]
		% third, delete unneeded array
		;	Stage=['$unset', array(VarKeys)]
		% unwind all solutions from disjunction
		;	Stage=['$unwind', string('$next')]
		% finally project a facet result
		;	set_vars_(StepVars, Stage)
		% and unset the next field
		;	Stage=['$unset', string('next')]
		),
		Pipeline
	).

%%
% each goal in a disjunction compiles into a lookup expression,
% then results of different goals are merged together.
%
compile_disjunction([], _, _, []) :- !.
compile_disjunction(
		[Goal|Xs], CutVars, Context,
		[[Stage,Key]|Ys]) :-
	% create findall pattern from step variables
	% TODO: reconsider this
	option(step_vars(StepVars), Context),
	maplist(nth0(1), StepVars, Pattern),
	% ensure goal is a list
	(	is_list(Goal) -> Goal0=Goal
	;	comma_list(Goal, Goal0)
	),
	% add match command checking for all previous goals with cut having
	% no solutions (size=0)
	% TODO: change to use just one match command with larger document
	findall(match(size(CutVar,int(0))),
		member([_, CutVar],CutVars),
		CutMatches),
	append(CutMatches, Goal0, GoalExtended),
	% since step_var does not list CutVars, we need to add them here to context
	% such that they will be accessible in findall's lookup
	select(outer_vars(OuterVars), Context, Context0),
	append(OuterVars, CutVars, OuterVars0),
	% compile the step
	query_compiler:step_compile(
		findall(Pattern, GoalExtended, Var),
		[outer_vars(OuterVars0)|Context0],
		Stage),
	query_compiler:var_key(Var, Key),
	%
	(	has_cut(Goal) -> CutVars0=[[Key,Var]|CutVars]
	;	CutVars0=CutVars
	),
	compile_disjunction(Xs, CutVars0, Context, Ys).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% HELPER
%%%%%%%%%%%%%%%%%%%%%%%

%%
% copies variable assignments from nested field "next" to root of document
%
set_vars_(Vars, ['$set', SetVars]) :-
	findall([Key,Val],
		(	member([Key,_],Vars),
			atom_concat('$next.', Key, Val)
		),
		SetVars
	).

%%
has_cut('!') :- !.
has_cut(Goal) :-
	% list of goals
	is_list(Goal), !,
	member(call(SubGoal)),
	comma_list(SubGoal,List),
	memberchk('!',List).
has_cut(Goal) :-
	% conjunction
	comma_list(Goal,List),
	has_cut(List).

