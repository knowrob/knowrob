:- module(lang_lists, []).

:- use_module(library('db/mongo/compiler')).
:- use_module(library('db/mongo/query')).

% TODO: support more list commands
%:- query_command_add(memberchk).
%:- query_command_add(sort).
%:- query_command_add(reverse).
%:- query_command_add(list_to_set).
%:- query_command_add(max_list).
%:- query_command_add(min_list).
%:- query_command_add(sum_list).
%:- query_command_add(length).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% nth/3
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% register query commands
:- query_command_add(nth).

%%
% nth/3 exposes variables of the pattern.
%
query_compiler:step_var(
		nth(_Index, _List, Pattern),
		[Key, Var]) :-
	pattern_variables_(Pattern, Vars),
	member([Key, Var], Vars).

%%
% nth/3 retrieves an a document at given index
% from some array field.
%
query_compiler:step_compile(
		nth(Index, List, _Elem),
		Context,
		Pipeline) :-
	option(ask, Context), !,
	query_compiler:var_key(List, ListKey),
	atom_concat('$', ListKey, ListKey0),
	% compute steps of the aggregate pipeline
	findall(Step,
		% retrieve array element and store in 'next' field
		(	Step=['$set',['next', ['$arrayElemAt',
					[string(ListKey0),integer(Index)]]]]
		% compute the intersection of scope so far with scope of next document
		;	scope_step(Context, Step)
		% project new variable groundings (the ones referred to in pattern)
		;	set_vars_(Context, ListKey, Step)
		% remove the next field again
		;	Step=['$unset', string('next')]
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% member/2
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% register query commands
:- query_command_add(member).

%%
% member exposes variables of the pattern.
%
query_compiler:step_var(
		member(Pattern, _List),
		[Key, Var]) :-
	pattern_variables_(Pattern, Vars),
	member([Key, Var], Vars).

%%
% member(Pattern,List) unwinds a list variable holding documents
% and exposes variables in Pattern to the rest of the pipeline.
%
query_compiler:step_compile(
		member(_Pattern, List),
		Context,
		Pipeline) :-
	option(ask, Context), !,
	query_compiler:var_key(List, ListKey),
	atom_concat('$', ListKey, ListKey0),
	% compute steps of the aggregate pipeline
	findall(Step,
		% copy the list to the next field for unwinding
		(	Step=['$set',['next', string(ListKey0)]]
		% at this point 'next' field holds an array of matching documents
		% that is unwinded here.
		;	Step=['$unwind',string('$next')]
		% compute the intersection of scope so far with scope of next document
		;	scope_step(Context, Step)
		% project new variable groundings (the ones referred to in pattern)
		;	set_vars_(Context, ListKey, Step)
		% remove the next field again
		;	Step=['$unset', string('next')]
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
set_vars_(Context, ListKey, ['$set', SetVars]) :-
	memberchk(step_vars(QueryVars), Context),
	memberchk(outer_vars(OuterVars), Context),
	memberchk([ListKey, list(_,Pattern)], OuterVars),
	pattern_variables_(Pattern,ListVars),
	set_vars_1(QueryVars, ListVars, SetVars).

%%
set_vars_1([], [], []) :- !.
set_vars_1([X|Xs], [Y|Ys], [Z|Zs]) :-
	X=[Key,_],
	Y=[ListKey,_],
	atom_concat('$next.', ListKey, Val),
	Z=[Key,string(Val)],
	set_vars_1(Xs, Ys, Zs).

%%
pattern_variables_(Pattern, Vars) :-
	term_variables(Pattern, PatternVars),
	pattern_variables_1(PatternVars, Vars).

pattern_variables_1([], []) :- !.
pattern_variables_1([X|Xs], [[Key,X]|Ys]) :-
	query_compiler:var_key(X,Key),
	pattern_variables_1(Xs, Ys).
