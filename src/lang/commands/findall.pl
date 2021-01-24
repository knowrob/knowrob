:- module(lang_findall, []).

:- use_module(library('db/mongo/client'),
		[ mng_one_db/2 ]).
:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(findall, [ask]).

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
		Context, Pipeline) :-
	% option(mode(ask), Context),
	findall(Step,
		% perform findall, collect results in 'next' array
		(	query_compiler:lookup_next_array(Terminals,
				[], [], Context, _, Step)
		% $set the list variable field from 'next' field
		;	set_result_(Pattern, List, Step)
		% array at 'next' field not needed anymore
		;	Step=['$unset', string('next')]
		),
		Pipeline).

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
	%
	findall([Key, string(Val)],
		(	(	Key='v_scope', Val='$$this.v_scope' )
		;	(	member(Var,PatternVars),
				query_compiler:var_key(Var, Key),
				atom_concat('$$this.', Key, Val)
			)
		),
		ElemProjection).
