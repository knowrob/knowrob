:- module(lang_findall, []).

:- use_module(library('db/mongo/compiler')).
:- use_module(library('db/mongo/query')).

%% register query commands
:- add_query_command(findall).

%%
query_compiler:step_expand(
		findall(Pattern, Goal, List),
		findall(Pattern, Expanded, List),
		Context) :-
	query_expand(Goal, Expanded, Context).

%%
% findall only exposes the List variable to the outside.
%
query_compiler:step_var(
		findall(Pattern, _, List),
		[List_var, list(List,Pattern)]) :-
	query_compiler:var_key(List, List_var).

%%
% findall(Pattern,Terminals,List) builds a list from matching
% documents. Pattern is a variable or list with variables
% referred to in Terminals list.
%
query_compiler:step_compile(
		findall(Pattern, Terminals, List),
		Context,
		[ Lookup, SetList, UnsetNext ]) :-
	option(ask, Context), !,
	% perform findall, collect results in 'next' array
	lookup_(Terminals, Context, Lookup),
	% $set the list variable field from 'next' field
	set_result_(Pattern, List, SetList),
	% array at 'next' field not needed anymore
	UnsetNext=['$unset', string('next')].

%%
lookup_(Terminals, Context, Lookup) :-
	% get variables referred to before
	option(outer_vars(OuterVars), Context),
	% join collection with single document
	mng_one_db(_DB, Coll),
	% generate inner pipeline first,
	% need to get access to variables for computing a join
	query_compile(Terminals,
		pipeline(Pipeline, InnerVars),
		Context),
	% pass variables from outer scope to inner if they are referred to
	% in the inner scope.
	% this is done through the *let* field of $lookup. e.g. when there is a
	% var "v_fg8t4" in the outer scope, we generate: ['let', ['v_fg8t4', '$v_fg8t4']].
	findall(Var,
		(	member([Var,_], InnerVars),
			member([Var,_], OuterVars)
		),
		SharedVars),
	findall([X,string(X0)],
		(	member(X, SharedVars),
			atom_concat('$',X,X0)
		),
		LetDoc),
	% first step: set all let variables so that they can be accessed
	% without aggregate operators in Pipeline, e.g.
	%     ['$set', ['v_fg8t4', '$$v_fg8t4']]
	findall([Y,string(Y0)],
		(	member(Y, SharedVars),
			atom_concat('$$',Y,Y0)
		),
		SetVars),
	%
	(	SetVars=[]
	->	Pipeline0=Pipeline
	;	Pipeline0=[['$set', SetVars] | Pipeline]
	),
	% finally compose the lookup document
	Lookup=['$lookup', [
		['from',string(Coll)],
		% create a field "next" with all matching documents
		['as',string('next')],
		% make fields from input document accessible in pipeline
		['let',LetDoc],
		% get matching documents
		['pipeline', array(Pipeline0)]
	]].

%%
% findall $set receives a list of matching documents in "next" field.
% $set uses additional $map operation to only keep the fields of
% variables referred to in Pattern.
%
set_result_(Pattern, List,
	['$set',
		[List_Key, ['$map',[
			['input',string('$next')],
			['in', ElemProjection]
		]]]
	]) :-
	query_compiler:var_key(List, List_Key),
	term_variables(Pattern, PatternVars),
	set_result_1(PatternVars, ElemProjection).

%%
% result is an array of documents that assigns a value to each
% variable in findall pattern.
% using array of documents is important to allow unwinding
% the array later.
%
set_result_1(L, X) :-
	findall([Key, string(Val)],
		(	(	Key='v_scope', Val='$$this.v_scope' )
		;	(	member(Var,L),
				query_compiler:var_key(Var, Key),
				atom_concat('$$this.', Key, Val)
			)
		),
		X).
