:- module(list_commands, []).

:- use_module(library('lang/compiler')).
:- use_module('intersect',
		[ mng_scope_intersect/5 ]).

%% query commands
:- query_compiler:add_command(length).
:- query_compiler:add_command(max_list).
:- query_compiler:add_command(min_list).
:- query_compiler:add_command(sum_list).
:- query_compiler:add_command(member).
:- query_compiler:add_command(memberchk).
:- query_compiler:add_command(nth0).
:- query_compiler:add_command(list_to_set).
%:- query_compiler:add_command(sort).
%:- query_compiler:add_command(reverse).

%% member(?Elem, +List)
% True if Elem is a member of List. 
% NOTE: here list cannot be a variable which is allowed in SWI Prolog.
%       Prolog then generates all possible list with Elem as member.
%
query_compiler:step_expand(member(Elem, List), Expanded, Context) :-
	query_expand((
		ground(List),
		% get size of list
		length(List, Size),
		Size0 is Size - 1,
		% iterate over the list
		between(0, Size0, Index),
		% unify Elem using nth0/3
		nth0(Index, List, Elem)
	), Expanded, Context).

%% memberchk(?Elem, +List)
% True when Elem is an element of List. This variant of member/2 is
% semi deterministic and typically used to test membership of a list. 
%
query_compiler:step_expand(memberchk(Elem, List), Expanded, Context) :-
	query_expand(limit(1, member(Elem,List)), Expanded, Context).

%% length(+List, ?Length)
% True if Length represents the number of elements in List.
%
query_compiler:step_compile(length(List, Length), _, []) :-
	ground(List),!,
	length(List, Length).

query_compiler:step_compile(length(List, Length), Ctx, Pipeline) :-
	% FIXME: SWI Prolog allows var(List), then yields lists with variables
	%          as elements. It is also allowed that both args are variables.
	%          then SWIPL generates infinite number of lists.
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
	Step=['$set', [SetKey, ['$setUnion', array([List0])]]].

%% nth0(?Index, +List, ?Elem)
% True when Elem is the Index’th element of List. Counting starts at 0. 
%
% NOTE: List must be kwown here. SWI Prolog also allows that List is a var.
%
query_compiler:step_compile(
		nth0(Index, List, Elem),
		Ctx, Pipeline) :-
	% TODO: below is a bit redundant with unification.pl
	%		- it also does not handle var-var bindings!
	%
	query_compiler:var_key_or_val(Index,Ctx,Index0),
	query_compiler:var_key_or_val(List,Ctx,List0),
	query_compiler:var_key_or_val(Elem,Ctx,Elem0),
	% compute steps of the aggregate pipeline
	findall(Step,
		(	query_compiler:set_if_var(Elem,
				['$arrayElemAt', array([List0,Index0])], Ctx, Step)
		;	query_compiler:set_if_var(Index,
				['$indexOfArray', array([List0,Elem0])], Ctx, Step)
		% unify terms
		;	Step=['$set', ['t_term1', Elem0]]
		;	Step=['$set', ['t_term2', ['$arrayElemAt', array([List0,Index0])]]]
		% assign vars in term1 to values of arguments in term2
		;	lang_unification:set_term_arguments('t_term1', 't_term2', Step)
		% perform equality test
		;	query_compiler:match_equals(string('$t_term1'), string('$t_term2'), Step)
		% project new variable groundings
		;	lang_unification:set_term_vars(Elem, 't_term1', Ctx, Step)
		% and cleanup
		;	Step=['$unset', array([string('t_term1'),string('t_term2')])]
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
		(	Step=['$set', ['t_val', [Operator, array([List0])]]]
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

test('findall+max_list'):-
	lang_query:test_command(
		(	findall(X,
				X is Num + 5,
				NumberList),
			max_list(NumberList, Max)
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

test('list_to_set(+Numbers)'):-
	lang_query:test_command(
		(	X is Num + 5,
			list_to_set([X,X], Set)
		),
		Num, double(4.5)),
	assert_equals(Set, [9.5]).

test('nth0(+Numbers)'):-
	lang_query:test_command(
		(	X is Num + 5,
			Y is Num * 2,
			nth0(1, [X,Y], Second)
		),
		Num, double(4.5)),
	assert_equals(Second, 9.0).

test('member(+Numbers)'):-
	findall(Val,
		lang_query:test_command(
			(	X is Num + 5,
				Y is Num * 2,
				member(Val, [X,Y])
			),
			Num, double(4.5)),
		Results),
	assert_equals(Results,[9.5,9.0]).

test('findall+member'):-
	findall(Val,
		lang_query:test_command(
			(	findall(X,
					((X is Num + 5);(X is Num * 2)),
					List),
				member(Val, List)
			),
			Num, double(4.5)),
		Results),
	assert_equals(Results,[9.5,9.0]).

%%:- query_compiler:add_command(memberchk,   [ask]).
%%:- query_compiler:add_command(sort,        [ask]).
%%:- query_compiler:add_command(reverse,     [ask]).

:- end_tests('list_commands').
