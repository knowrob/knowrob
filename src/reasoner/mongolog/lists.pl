:- module(mongolog_lists, []).
/** <module> List manipulation in mongolog programs.

The following predicates are supported:

| Predicate     | Arguments |
| ---           | ---       |
| length/2      | +List, ?Length |
| max_list/2    | +List, ?Max |
| min_list/2    | +List, ?Min |
| sum_list/2    | +List, ?Sum |
| member/2      | ?Elem, +List |
| memberchk/2   | ?Elem, +List |
| nth0/3        | ?Index, +List, ?Elem |
| list_to_set/2 | +List, -Set |
| sort/2        | +List, -Sorted |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=allsolutions
@license BSD
*/

:- use_module('mongolog').

%% query commands
:- mongolog:add_command(length).
:- mongolog:add_command(max_list).
:- mongolog:add_command(min_list).
:- mongolog:add_command(sum_list).
:- mongolog:add_command(member).
:- mongolog:add_command(memberchk).
:- mongolog:add_command(nth0).
:- mongolog:add_command(list_to_set).
:- mongolog:add_command(sort).

%% member(?Elem, +List)
% True if Elem is a member of List. 
% NOTE: here list cannot be a variable which is allowed in SWI Prolog.
%       Prolog then generates all possible list with Elem as member.
%
mongolog:step_expand(member(Elem, List), Expanded) :-
	mongolog_expand((
		% get size of list
		length(List, Size),
		Size0 is Size - 1,
		% iterate over the list
		between(0, Size0, Index),
		% unify Elem using nth0/3
		nth0(Index, List, Elem)
	), Expanded).

%% memberchk(?Elem, +List)
% True when Elem is an element of List. This variant of member/2 is
% semi deterministic and typically used to tests membership of a list.
%
mongolog:step_expand(memberchk(Elem, List), Expanded) :-
	mongolog_expand(limit(1, member(Elem,List)), Expanded).

%% length(+List, ?Length)
% True if Length represents the number of elements in List.
%
mongolog:step_compile(length(List, Length), _, []) :-
	ground(List),!,
	length(List, Length).

mongolog:step_compile(length(List, Length), Ctx, Pipeline) :-
	compile_list_attribute(List, Length, '$size', Ctx, Pipeline).

%% max_list(+List:list(number), -Max:number)
% True if Max is the largest number in List. Fails if List is empty. 
%
mongolog:step_compile(max_list(List, Max), _, []) :-
	ground(List),!,
	max_list(List, Max).

mongolog:step_compile(max_list(List, Max), Ctx, Pipeline) :-
	compile_list_attribute(List, Max, '$max', Ctx, Pipeline).

%% min_list(+List, ?Min)
% True if Min is the smallest number in List. Fails if List is empty.
%
mongolog:step_compile(min_list(List, Min), _, []) :-
	ground(List),!,
	min_list(List, Min).

mongolog:step_compile(min_list(List, Min), Ctx, Pipeline) :-
	compile_list_attribute(List, Min, '$min', Ctx, Pipeline).

%% sum_list(+List, -Sum)
% Sum is the result of adding all numbers in List.
%
mongolog:step_compile(sum_list(List, Sum), _, []) :-
	ground(List),!,
	sum_list(List, Sum).

mongolog:step_compile(sum_list(List, Sum), Ctx, Pipeline) :-
	compile_list_attribute(List, Sum, '$sum', Ctx, Pipeline).

%% list_to_set(+List, -Set)
% Removes duplicates from a list.
% List may *not* contain variables when this is evaluated.
%
mongolog:step_compile(list_to_set(List, Set), _, []) :-
	ground(List),!,
	list_to_set(List, Set).

mongolog:step_compile(
		list_to_set(List, Set), Ctx,
		[Step]) :-
	mongolog:var_key_or_val(List,Ctx,List0),
	mongolog:var_key(Set,Ctx,SetKey),
	Step=['$set', [SetKey, ['$setUnion', array([List0])]]].

%% sort(+List, -Sorted)
% True if Sorted can be unified with a list holding the elements of List,
% sorted to the standard order in mongo. Duplicates are removed.
%
mongolog:step_compile(
		sort(List, Sorted), Ctx, Pipeline) :-
	mongolog:var_key_or_val(List,Ctx,List0),
	mongolog:var_key(Sorted,Ctx,SortedKey),
	mongolog_one_db(_DB, Coll),
	% compute steps of the aggregate pipeline
	findall(Step,
		% assign list field and remove duplicates
		(	Step=['$set', ['t_list', ['$setUnion', array([List0])]]]
		% use lookup to sort the array by using $unwind+$sort in lookup pipeline
		;	Step=['$lookup', [
				['from', string(Coll)],
				['as', string('t_sorted')],
				['let', [['t_list', string('$t_list')]]],
				['pipeline', array([
					['$set', ['elem', string('$$t_list')]],
					['$unwind', string('$elem')],
					['$sort', ['elem', integer(1)]]
				])]
			]]
		% map from array of documents to array of values
		;	Step=['$set', [SortedKey, ['$map', [
				['input', string('$t_sorted')],
				['in', string('$$this.elem')]
			]]]]
		;	Step=['$unset', array([string('t_list'), string('t_sorted')])]
		),
		Pipeline
	).

%% nth0(?Index, +List, ?Elem)
% True when Elem is the Index’th element of List. Counting starts at 0. 
%
% NOTE: List must be kwown here. SWI Prolog also allows that List is a var.
%
mongolog:step_compile(
		nth0(Index, List, Elem),
		Ctx, Pipeline) :-
	% TODO: below is a redundant with unification.pl
	%		- it also does not handle var-var bindings!
	%
	mongolog:var_key_or_val(Index,Ctx,Index0),
	mongolog:var_key_or_val(List,Ctx,List0),
	mongolog:var_key_or_val(Elem,Ctx,Elem0),
	% compute steps of the aggregate pipeline
	findall(Step,
		(	mongolog:set_if_var(Elem,
				['$arrayElemAt', array([List0,Index0])], Ctx, Step)
		;	mongolog:set_if_var(Index,
				['$indexOfArray', array([List0,Elem0])], Ctx, Step)
		% unify terms
		;	Step=['$set', ['t_term1', Elem0]]
		;	Step=['$set', ['t_term2', ['$arrayElemAt', array([List0,Index0])]]]
		% assign vars in term1 to values of arguments in term2
		;	mongolog_unification:set_term_arguments('t_term1', 't_term2', Step)
		% perform equality tests
		;	mongolog:match_equals(string('$t_term1'), string('$t_term2'), Step)
		% project new variable groundings
		;	mongolog_unification:set_term_vars(Elem, 't_term1', Ctx, Step)
		% and cleanup
		;	Step=['$unset', array([string('t_term1'),string('t_term2')])]
		),
		Pipeline
	).

%% +List, ?Attribute
% Applies an operator on grounded array (List) and unifies the
% second argument with the result (i.e. Attribute maybe ground or var).
%
compile_list_attribute(List, Attribute, Operator, Ctx, Pipeline) :-
	mongolog:var_key_or_val(List, Ctx, List0),
	mongolog:var_key_or_val(Attribute, Ctx, Attribute0),
	findall(Step,
		% first compute the attribute
		(	Step=['$set', ['t_val', [Operator, array([List0])]]]
		% then assign the value to the attribute if it is a variable
		;	mongolog:set_if_var(Attribute,    string('$t_val'), Ctx, Step)
		% then ensure that the attribute has the right value
		;	mongolog:match_equals(Attribute0, string('$t_val'), Step)
		% finally remove temporary field again
		;	Step=['$unset', string('t_val')]
		),
		Pipeline
	).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_lists').

test('length(+,+)'):-
	assert_true(mongolog:test_call(
		length(List, 2), List, [2,4])),
	assert_true(mongolog:test_call(
		length([2,4], Count), Count, 2)),
	assert_true(mongolog:test_call(
		length([], Count), Count, 0)),
	assert_false(mongolog:test_call(
		length(List, 3), List, [2,4])),
	assert_false(mongolog:test_call(
		length([2,4], Count), Count, 3)).

test('length(+,-)'):-
	mongolog:test_call(
		length(List, Length), List, [2,4]),
	assert_equals(Length, 2).

test('findall+length'):-
	mongolog:test_call(
		(	findall(X,
				((X is Num + 5);(X is Num * 2)),
				List),
			length(List, Length)
		),
		Num, double(4.5)),
	assert_equals(Length, 2).

test('findall+length[conditional]'):-
	mongolog:test_call(
		(	findall(X,
				((Num < 4.0, X is Num + 5);(Num > 4.0, X is Num * 2)),
				List),
			length(List, Length)
		),
		Num, double(4.5)),
	assert_equals(Length,1).

test('max_list(+Numbers)'):-
	mongolog:test_call(
		(	X is Num + 5,
			Y is Num * 2,
			max_list([X,Y], Max)
		),
		Num, double(4.5)),
	assert_equals(Max, 9.5).

test('findall+max_list'):-
	mongolog:test_call(
		(	findall(X,
				X is Num + 5,
				NumberList),
			max_list(NumberList, Max)
		),
		Num, double(4.5)),
	assert_equals(Max, 9.5).

test('min_list(+Numbers,-Min)'):-
	mongolog:test_call(
		(	X is Num + 5,
			Y is Num * 2,
			min_list([X,Y], Min)
		),
		Num, double(4.5)),
	assert_equals(Min, 9.0).

test('sum_list(+Numbers,-Sum)'):-
	mongolog:test_call(
		(	X is Num + 5,
			Y is Num * 2,
			sum_list([X,Y], Sum)
		),
		Num, double(4.5)),
	assert_equals(Sum, 18.5).

test('list_to_set(+Numbers)'):-
	mongolog:test_call(
		(	X is Num + 5,
			list_to_set([X,X], Set)
		),
		Num, double(4.5)),
	assert_equals(Set, [9.5]).

test('sort(+Numbers)'):-
	mongolog:test_call(
		(	X is Num + 5,
			sort([4,X,Num,2], Sorted)
		),
		Num, double(4)),
	assert_equals(Sorted, [2.0, 4.0, 9.0]).

test('sort(+Atoms)'):-
	mongolog:test_call(
		sort([d,a,X,b], Sorted), X, string(s)),
	assert_equals(Sorted, [a,b,d,s]).

test('sort(+AtomsAndNumbers)'):-
	mongolog:test_call(
		sort([9,a,X,7], Sorted), X, string(s)),
	assert_equals(Sorted, [7.0,9.0,a,s]).

test('nth0(+Numbers)'):-
	mongolog:test_call(
		(	X is Num + 5,
			Y is Num * 2,
			nth0(1, [X,Y], Second)
		),
		Num, double(4.5)),
	assert_equals(Second, 9.0).

test('member(+Number)'):-
	findall(Val,
		mongolog:test_call(
			(	X is Num + 5,
				member(Val, [X])
			),
			Num, double(4.5)),
		Results),
	assert_equals(Results,[9.5]).

test('member(+Numbers)'):-
	findall(Val,
		mongolog:test_call(
			(	X is Num + 5,
				Y is Num * 2,
				member(Val, [X,Y])
			),
			Num, double(4.5)),
		Results),
	assert_equals(Results,[9.5,9.0]).

test('findall+nth0'):-
	mongolog:test_call(
		(	findall(X,
				((X is Num + 5);(X is Num * 2)),
				List),
			nth0(0, List, Val)
		),
		Num, double(4.5)),
	assert_equals(Val,9.5).

test('findall+member'):-
	findall(Val,
		mongolog:test_call(
			(	findall(X,
					((X is Num + 5);(X is Num * 2)),
					List),
				member(Val, List)
			),
			Num, double(4.5)),
		Results),
	assert_equals(Results,[9.5,9.0]).

test('(fail;findall)+length'):-
	mongolog:test_call(
		(	( (	( Num < 4.0 )
			;	( Num > 4.0, findall(X,
					((X is Num + 5);(X is Num * 2)),
					InnerList) )
			)),
			assign(List, InnerList)
		),
		Num, double(4.5)),
	assert_equals(InnerList,[9.5,9.0]),
	assert_equals(List,[9.5,9.0]).

:- end_tests('mongolog_lists').
