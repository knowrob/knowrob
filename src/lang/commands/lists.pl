:- module(list_commands, []).

:- use_module(library('lang/compiler')).
:- use_module('intersect',
		[ mng_scope_intersect/5 ]).

% TODO: handling of patterns is not nice at the moment.
%          - it is limiting that elements in lists must all have same pattern
%          - probably would need better handling of terms.

%% query commands
:- query_compiler:add_command(length,      [ask]).
:- query_compiler:add_command(max_list,    [ask]).
:- query_compiler:add_command(min_list,    [ask]).
:- query_compiler:add_command(sum_list,    [ask]).
:- query_compiler:add_command(member,      [ask]).
%:- query_compiler:add_command(memberchk,   [ask]).
:- query_compiler:add_command(nth,         [ask]).
:- query_compiler:add_command(list_to_set, [ask]).
%:- query_compiler:add_command(sort,        [ask]).
%:- query_compiler:add_command(reverse,     [ask]).

%% query variables
query_compiler:step_var(length(A,B),      Ctx, Var) :- query_compiler:get_var([A,B], Ctx, Var).
query_compiler:step_var(max_list(A,B),    Ctx, Var) :- query_compiler:get_var([A,B], Ctx, Var).
query_compiler:step_var(min_list(A,B),    Ctx, Var) :- query_compiler:get_var([A,B], Ctx, Var).
query_compiler:step_var(sum_list(A,B),    Ctx, Var) :- query_compiler:get_var([A,B], Ctx, Var).
query_compiler:step_var(list_to_set(A,B), Ctx, Var) :- query_compiler:get_var([A,B], Ctx, Var).

query_compiler:step_var(nth(List, N, Pattern), Ctx, Var) :-
	(	pattern_var_key(Pattern, Ctx, Var)
	;	query_compiler:get_var([List,N], Ctx, Var)
	).

query_compiler:step_var(member(Elem, List), Ctx, Var) :-
	(	pattern_var_key(Elem, Ctx, Var)
	;	query_compiler:get_var([List], Ctx, Var)
	).

%% length(+List, ?Length)
% True if Length represents the number of elements in List.
%
query_compiler:step_compile(length(List, Length), _, []) :-
	ground(List),!,
	length(List, Length).

query_compiler:step_compile(length(List, Length), Ctx, Pipeline) :-
	% FIXME: SWI Prolog allows var(List), then yields lists with variables
	%          as elements. It is also allowed that both args are variables.
	%          the SWIPL generates infinite number of lists.
	compile_list_attribute(List, Length, '$size', Ctx, Pipeline).

%% max_list(+List:list(number), -Max:number)
% True if Max is the largest number in List. Fails if List is empty. 
%
query_compiler:step_compile(max_list(List, Max), _, []) :-
	ground(List),!,
	max_list(List, Max).

query_compiler:step_compile(max_list(List, Max), Ctx, Pipeline) :-
	compile_list_attribute(List, Max, '$max', Ctx, Pipeline).

%% min_list(+List, ?Min)
% True if Min is the smallest number in List. Fails if List is empty.
%
query_compiler:step_compile(min_list(List, Min), _, []) :-
	ground(List),!,
	min_list(List, Min).

query_compiler:step_compile(min_list(List, Min), Ctx, Pipeline) :-
	compile_list_attribute(List, Min, '$min', Ctx, Pipeline).

%% sum_list(+List, -Sum)
% Sum is the result of adding all numbers in List.
%
query_compiler:step_compile(sum_list(List, Sum), _, []) :-
	ground(List),!,
	sum_list(List, Sum).

query_compiler:step_compile(sum_list(List, Sum), Ctx, Pipeline) :-
	compile_list_attribute(List, Sum, '$sum', Ctx, Pipeline).

%% list_to_set(+List, -Set)
% Removes duplicates from a list.
% List may *not* contain variables when this is evaluated.
%
query_compiler:step_compile(list_to_set(List, Set), _, []) :-
	ground(List),!,
	list_to_set(List, Set).

query_compiler:step_compile(
		list_to_set(List, Set), Ctx,
		[Step]) :-
	% FIXME: SWI Prolog allows ground(Set)
	% FIXME: Set and List have same ordering in SWI Prolog, but mongo does not ensure this.
	query_compiler:var_key_or_val(List,Ctx,List0),
	query_compiler:var_key(Set,Ctx,SetKey),
	Step=['$addToSet', [SetKey, ['$each', List0]]].


%% member(?Elem, ?List)
% True if Elem is a member of List. 
%
query_compiler:step_compile(
		member(_Elem, List),
		Ctx, Pipeline) :-
	% option(mode(ask), Context),
	query_compiler:var_key(List, Ctx, ListKey),
	atom_concat('$', ListKey, ListKey0),
	% compute steps of the aggregate pipeline
	findall(Step,
		% copy the list to the next field for unwinding
		(	Step=['$set',['next', string(ListKey0)]]
		% at this point 'next' field holds an array of matching documents
		% that is unwinded here.
		;	Step=['$unwind',string('$next')]
		% compute the intersection of scope so far with scope of next document
		;	mng_scope_intersect('v_scope',
				string('$next.scope.time.since'),
				string('$next.scope.time.until'),
				Ctx, Step)
		% project new variable groundings (the ones referred to in pattern)
		;	set_vars_(Ctx, ListKey, Ctx, Step)
		% remove the next field again
		;	Step=['$unset', string('next')]
		),
		Pipeline
	).

%%
% nth/3 retrieves an a document at given index
% from some array field.
% TODO: harmonize this with SWIPL nth predicates
%
query_compiler:step_compile(
		nth(Index, List, _Elem),
		Ctx, Pipeline) :-
	% option(mode(ask), Context),
	query_compiler:var_key(List, Ctx, ListKey),
	atom_concat('$', ListKey, ListKey0),
	% compute steps of the aggregate pipeline
	findall(Step,
		% retrieve array element and store in 'next' field
		(	Step=['$set',['next', ['$arrayElemAt',
					[string(ListKey0),integer(Index)]]]]
		% compute the intersection of scope so far with scope of next document
		;	mng_scope_intersect('v_scope',
				string('$next.scope.time.since'),
				string('$next.scope.time.until'),
				Ctx, Step)
		% project new variable groundings (the ones referred to in pattern)
		;	set_vars_(Ctx, ListKey, Step)
		% remove the next field again
		;	Step=['$unset', string('next')]
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% +List, ?Attribute
% Applies an operator on grounded array (List) and unifies the
% second argument with the result (i.e. Attribute maybe ground or var).
%
compile_list_attribute(List, Attribute, Operator, Ctx, Pipeline) :-
	query_compiler:var_key_or_val(List, Ctx, List0),
	query_compiler:var_key_or_val(Attribute, Ctx, Attribute0),
	findall(Step,
		% first compute the attribute
		(	Step=['$set', ['t_val', [Operator, List0]]]
		% then assign the value to the attribute if it is a variable
		;	query_compiler:set_if_var(Attribute,    string('$t_val'), Ctx, Step)
		% then ensure that the attribute has the right value
		;	query_compiler:match_equals(Attribute0, string('$t_val'), Step)
		% finally remove temporary field again
		;	Step=['$unset', string('t_val')]
		),
		Pipeline
	).

%%
set_vars_(Context, ListKey, Ctx, ['$set', SetVars]) :-
	memberchk(step_vars(QueryVars), Context),
	memberchk(outer_vars(OuterVars), Context),
	memberchk([ListKey, list(_,Pattern)], OuterVars),
	pattern_variables_(Pattern,Ctx,ListVars),
	set_vars_1(QueryVars, ListVars, SetVars).

%%
set_vars_1([], [], []) :- !.
set_vars_1([X|Xs], [Y|Ys], [Z|Zs]) :-
	% FIXME: need condition $set here? i.e. because a variable is not grounded
	X=[Key,_],
	Y=[ListKey,_],
	atom_concat('$next.', ListKey, Val),
	Z=[Key,string(Val)],
	set_vars_1(Xs, Ys, Zs).

%%
pattern_var_key(Pattern, Ctx, [Key, Var]) :-
	pattern_variables_(Pattern, Ctx, Vars),
	member([Key, Var], Vars).

%%
pattern_variables_(Pattern, Ctx, Vars) :-
	term_variables(Pattern, PatternVars),
	pattern_variables_1(PatternVars, Ctx, Vars).

pattern_variables_1([], _, []) :- !.
pattern_variables_1([X|Xs], Ctx, [[Key,X]|Ys]) :-
	query_compiler:var_key(X,Ctx,Key),
	pattern_variables_1(Xs,Ctx,Ys).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('list_commands').

test('findall+length'):-
	lang_query:test_command(
		(	findall(X,
				((X is Num + 5);(X is Num * 2)),
				List),
			length(List, Length)
		),
		Num, double(4.5)),
	assert_equals(Length, 2).

test('max_list(+Numbers)'):-
	lang_query:test_command(
		(	X is Num + 5,
			Y is Num * 2,
			max_list([X,Y], Max)
		),
		Num, double(4.5)),
	assert_equals(Max, 9.5).

test('min_list(+Numbers)'):-
	lang_query:test_command(
		(	X is Num + 5,
			Y is Num * 2,
			min_list([X,Y], Max)
		),
		Num, double(4.5)),
	assert_equals(Max, 9.0).

test('sum_list(+Numbers)'):-
	lang_query:test_command(
		(	X is Num + 5,
			Y is Num * 2,
			sum_list([X,Y], Max)
		),
		Num, double(4.5)),
	assert_equals(Max, 18.5).

test('findall+max_list'):-
	lang_query:test_command(
		(	findall(X,
				X is Num + 5,
				NumberList),
			max_list(NumberList, Max)
		),
		Num, double(4.5)),
	assert_equals(Max, 9.5).

%:- query_compiler:add_command(member,      [ask]).
%%:- query_compiler:add_command(memberchk,   [ask]).
%:- query_compiler:add_command(nth,         [ask]).
%:- query_compiler:add_command(list_to_set, [ask]).
%%:- query_compiler:add_command(sort,        [ask]).
%%:- query_compiler:add_command(reverse,     [ask]).

:- end_tests('list_commands').
