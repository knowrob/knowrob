:- module(mng_term_facet, []).

:- use_module(library('db/mongo/lang/compiler')).
:- use_module(library('db/mongo/lang/query')).

%% register query commands
:- mng_query_command(facet).

%%
% Each facet is a KnowRob language expression and needs to be expanded.
%
mng_query:step_expand(
		facet([]),
		facet([]), _Context).
%mng_query:step_expand(
%		facet([step(X,Y)|Xs]),
%		facet([step(X,Y)|Ys]), Context) :-
%	mng_query:step_expand(facet(Xs), facet(Ys), Context).
mng_query:step_expand(
		facet([X|Xs]),
		facet([Y|Ys]), Context) :-
	mng_expand(X,Y,Context),
	mng_query:step_expand(facet(Xs), facet(Ys), Context).

%%
% expose variables to the outside that appear in every facet.
% NOTE: thus it is not possible to to get the value of variables that
%       appear only in some facets.
%
mng_compiler:step_var(facet([First|Rest]), Var) :-
	% make choicepoint for each variable in First
	mng_compiler:step_var(First, Var),
	% only proceed if Var is a variable in each facet
	forall(
		member(Y, Rest),
		mng_compiler:step_var(Y, Var)
	).

%%
% facet(Facets) creates a disjunction of aggregate pipelines.
% unfortunately mongo does not support this as of now.
% so we proceed as follows:
% 1. for each facet, generate findall/3 expression and store this into some array field
% 2. then concat all these array fields into single array stored in field 'next'
% 3. unwind the next array
%
% TODO: seems like a good usecase for map-reduce?
%
mng_compiler:step_compile(
		facet(Facets),
		Context,
		Pipeline) :-
	option(step_vars(StepVars)),
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
% each facet compiles into an aggregate pipeline without restrictions
% the result is written into a list, which is accomplished by a findall command.
%
compile_facet_(StepVars, Facet, Pipeline, VarKey, Context) :-
	% create findall pattern from step variables
	maplist(nth0(1), StepVars, Pattern),
	% compile the step
	mng_compiler:step_compile(
		findall(Pattern, Facet, Var),
		Context,
		Pipeline),
	mng_compiler:var_key(Var, VarKey).
