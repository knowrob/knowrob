:- module(control_commands, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(true,  [ask,tell]).
:- query_compiler:add_command(false, [ask,tell]).
:- query_compiler:add_command(fail,  [ask,tell]).
:- query_compiler:add_command('\\+', [ask]).
:- query_compiler:add_command('!',   [ask,tell]).
:- query_compiler:add_command(';',   [ask]).


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

%%
% Expand each goal of a conjunction.
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
query_compiler:step_compile('!', Context, Pipeline) :-
	% TODO: cut operator
	%     -- 1. within a clause `cut` wraps terms before
	%             in a lookup with $limit
	%     -- 2. disjunctions keep track of cut in disjuncts
	%             and does not proceed if cut+matching docs in previous step
	fail.

%% TODO: :Condition -> :Action
% If-then and If-Then-Else.
% The ->/2 construct commits to the choices made at its left-hand side,
% destroying choice points created inside the clause (by ;/2),
% or by goals called by this clause.
%
% FIXME: semantics is different when embedded in ;, because the other clause
%            must be pruned by cut here
%
%'->'(If,Then) ?> call(If), !, call(Then).
% TODO: support *-> operator, seems difficult
%'*->'(If,Then) ?> call(If), call(Then).
query_compiler:step_compile(';'('->'(A,B),C), Context, Pipeline) :-
	fail.
query_compiler:step_compile('->'(A,B), Context, Pipeline) :-
	fail.
query_compiler:step_compile('*->'(A,B), Context, Pipeline) :-
	fail.

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
	semicolon_list(';'(A,B), Facets),
	% read options from context
	% option(mode(ask), Context),
	option(step_vars(StepVars), Context),
	% get a list of tuples (pipeline, list variable key)
	% the result of each pipeline is written to a list,
	% and resulting lists are concatenated later to
	% achieve disjunction.
	% elements of the list are docments with variables
	% in StepVars being assigned.
	findall([FindallStage, VarKey0],
		(	member(Facet,Facets),
			compile_facet_(StepVars, Facet,
				FindallStage, VarKey0, Context)
		),
		FindallStages),
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
% each facet compiles into an aggregate pipeline without restrictions
% the result is written into a list, which is accomplished by a findall command.
%
compile_facet_(StepVars, Facet, Pipeline, VarKey, Context) :-
	% create findall pattern from step variables
	maplist(nth0(1), StepVars, Pattern),
	% compile the step
	query_compiler:step_compile(
		findall(Pattern, Facet, Var),
		Context, Pipeline),
	query_compiler:var_key(Var, VarKey).


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

