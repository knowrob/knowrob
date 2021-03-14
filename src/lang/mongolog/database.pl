:- module(mongolog_database,
		[ mongolog_add_predicate(+,+,+),
		  mongolog_drop_predicate(+)
		]).
/** <module> Storage of predicates in mongolog programs.

The following predicates are supported:

| Predicate    | Arguments |
| ---          | ---       |
| assert/1     | +Head |
| retractall/1 | +Head |

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=dynpreds
@license BSD
*/

:- use_module('mongolog').


%% Predicates that are stored in a mongo collection
:- dynamic mongolog_predicate/3.


%% query commands
:- mongolog:add_command(assert).
:- mongolog:add_command(retractall).


%% mongolog_predicate(+Term, -Fields, -Options) is semidet.
%
%
mongolog_predicate(Term, Fields, Options) :-
	compound(Term),!,
	Term =.. [Functor|_],
	mongolog_predicate(Functor, Fields, Options).


%% mongolog_add_predicate(+Functor, +Fields, +Options) is semidet.
%
% Register a predicate that stores facts in the database.
% Functor is the functor of a n-ary predicate, and Fields is
% a n-elemental list of keys associated to the different
% arguments of the predicate.
% Options is a list of optional paramers:
%
% | indices(List) | a list of indices passed to setup_collection/2 |
% | collection(Collection) | name of the collection where predicate is stored, default is to use the functor of the predicate |
%
% Current limitation: there cannot be predicates with the same functor,
% but different arity.
%
% @param Functor functor of the predicate
% @param Fields field names of predicate arguments
% @param Options option list
%
mongolog_add_predicate(Functor, _, _) :-
	mongolog_predicate(Functor, _, _),
	!,
	throw(permission_error(modify,database_predicate,Functor)).

mongolog_add_predicate(Functor, Fields, Options) :-
	setup_predicate_collection(Functor, Fields, Options),
	assertz(mongolog_predicate(Functor, Fields, Options)),
	mongolog:add_command(Functor).

%%
setup_predicate_collection(Functor, [FirstField|_], Options) :-
	% TODO support fields marked with -/+ here
	option(indices(Indices), Options, [[FirstField]]),
	setup_collection(Functor, Indices).


%% mongolog_drop_predicate(+Functor) is det.
%
% Delete all facts associated to predicate with
% given functor.
%
% @param Functor functor of the predicate
%
mongolog_drop_predicate(Functor) :-
	mng_get_db(DB, Collection, Functor),
	mng_drop(DB, Collection).

%%
mongolog:step_compile(assert(Term), Ctx, Pipeline, StepVars) :-
	merge_options([operation(assert),mode(tell)], Ctx, Ctx0),
	mongolog:step_compile(Term, Ctx0, Pipeline, StepVars).

mongolog:step_compile(retractall(Term), Ctx, Pipeline, StepVars) :-
	merge_options([operation(retractall)], Ctx, Ctx0),
	mongolog:step_compile(Term, Ctx0, Pipeline, StepVars).

%%
mongolog:step_compile(Term, Ctx, Pipeline, StepVars) :-
	% get predicate fields and options
	mongolog_predicate(Term, ArgFields, Options),
	% get predicate functor and arguments
	Term =.. [Functor|Args],
	% get the database collection of the predicate
	(	option(collection(Collection), Options)
	;	mng_get_db(_DB, Collection, Functor)
	),
	!,
	% read variable in Term
	mongolog:step_vars(Term, Ctx, StepVars0),
	mongolog:add_assertion_var(StepVars0, Ctx, StepVars),
	% add predicate options to compile context
	merge_options([
		step_vars(StepVars),
		collection(Collection)
	], Ctx, Ctx0),
	merge_options(Ctx0, Options, Ctx1),
	% zip field names with predicate arguments
	zip(ArgFields, Args, Zipped),
	% get aggregation pipeline
	(	option(operation(assert), Ctx)     -> mongolog_predicate_assert(Zipped, Ctx1, Pipeline)
	;	option(operation(retractall), Ctx) -> mongolog_predicate_retractall(Zipped, Ctx1, Pipeline)
	;	option(mode(tell), Ctx)            -> mongolog_predicate_assert(Zipped, Ctx1, Pipeline)
	;	mongolog_predicate_call(Zipped, Ctx1, Pipeline)
	).

%%
mongolog_predicate_call(Zipped, Ctx, Pipeline) :-
	unpack_compound(Zipped, Unpacked),
	findall(InnerStep,
		match_predicate(Unpacked, Ctx, InnerStep),
		InnerPipeline),
	%
	findall(Step,
		% look-up documents into 't_pred' array field
		(	lookup_predicate('t_pred', InnerPipeline, Ctx, Step)
		% unwind lookup results and assign variables
		;	Step=['$unwind', string('$t_pred')]
		;	project_predicate(Unpacked, Ctx, Step)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline).

%%
mongolog_predicate_retractall(Zipped, Ctx, Pipeline) :-
	option(collection(Collection), Ctx),
	unpack_compound(Zipped, Unpacked),
	findall(InnerStep,
		(	match_predicate(Unpacked, Ctx, InnerStep)
		% retractall first performs match, then only projects the id of the document
		;	project_retract(InnerStep)
		),
		InnerPipeline),
	%
	findall(Step,
		% look-up documents into 't_pred' array field
		(	lookup_predicate('t_pred', InnerPipeline, Ctx, Step)
		% add removed facts to assertions list
		;	mongolog:add_assertions(string('$t_pred'), Collection, Step)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline
	).

%%
mongolog_predicate_assert(Zipped, Ctx, Pipeline) :-
	option(collection(Collection), Ctx),
	% create a document
	findall([Field,Val],
		(	member([Field,Arg],Zipped),
			mongolog:var_key_or_val(Arg, Ctx, Val)
		),
		PredicateDoc),
	% and add it to the list of asserted documents
	findall(Step,
		mongolog:add_assertion(PredicateDoc, Collection, Step),
		Pipeline).

%%
unpack_compound([], []) :- !.

unpack_compound([X|Xs], Unpacked) :-
	unpack_compound1(X, Unpacked0),
	unpack_compound(Xs, Unpacked1),
	append(Unpacked0, Unpacked1, Unpacked).

unpack_compound1([Field,Arg], Unpacked) :-
	!, unpack_compound1([Field,Arg,[]], Unpacked).

unpack_compound1([Field,Arg,Is], [[Field,Arg,Is]]) :-
	(ground(Arg);var(Arg)),!.

unpack_compound1([Field,Term,Is], [[FunctorField,Functor,Is]|ArgsUnpacked]) :-
	compound(Term),!,
	% compound term with free variables
	Term =.. [Functor|Args],
	atom_concat(Field,'.value.functor',FunctorField),
	arg_fields_(Field, Args, Is, 0, ArgFields),
	unpack_compound(ArgFields, ArgsUnpacked).

%%
arg_fields_(_, [], _, []) :- !.
arg_fields_(Field, [X|Xs], Is, Index, [[Field,X,[Index|Is]]|Rest]) :-
	Index1 is Index+1,
	arg_fields_(Field, Xs, Index1, Rest).

%%
set_nested_args(Unpacked, ['$set', NestedArgs]) :-
	findall(X,
		(	member([Key,_,Is], Unpacked),
			set_nested_arg(Key,Is,X)
		),
		NestedArgs0),
	NestedArgs0 \== [],
	list_to_set(NestedArgs0, NestedArgs).

set_nested_arg(Key, [I|Is], Arg) :-
	atomic_list_concat(['$',Key,'.value.args'], '', ThisVal),
	atomic_list_concat([Key,I],'_',Y),
	(	Arg=[Y, ['$arrayElemAt', array([string(ThisVal),integer(I)])]]
	;	set_nested_arg(Y, Is, Y)
	).

%% $lookup
%
lookup_predicate(Field, InnerPipeline, Ctx, ['$lookup', [
		['from', string(Collection)],
		['as', string(Field)],
		['let', LetDoc],
		['pipeline', array(InnerPipeline)]
	]]) :-
	option(collection(Collection), Ctx),
	option(step_vars(StepVars), Ctx),
	mongolog:lookup_let_doc(StepVars, LetDoc).

%% $set
%
project_predicate(Unpacked, Ctx, Step) :-
	member([FieldPath0,Var,Is], Unpacked),
	mongolog:var_key(Var,Ctx,VarKey),
	atomic_list_concat([FieldPath0|Is],'_',FieldPath),
	atom_concat('$t_pred.', FieldPath, FieldQuery),
	Step=['$set', [VarKey, string(FieldQuery)]].

%%
project_retract(Step) :-
	(	Step=['$project',[['_id',int(1)]]]
	;	Step=['$set',['delete',bool(true)]]
	).

%% $match
%
match_predicate(Unpacked, Ctx, Match) :-
	findall(MatchQuery,
		% first match what is grounded compile-time
		(	(	findall([DocKey, ValueQuery],
					(	member([DocKey,Value,[]], Unpacked),
						mng_query_value(Value, ValueQuery)
					),
					MatchQuery),
				MatchQuery \== []
			)
		% next match variables grounded in call context
		;	(	member([DocKey,Var,[]], Unpacked),
				match_conditional(DocKey, Var, Ctx, MatchQuery)
			)
		),
		MatchQueries
	),
	(	MatchQueries=[FirstMatch]
	->	Match=['$match', FirstMatch]
	;	Match=['$match', ['$and', array(MatchQueries)]]
	).

match_predicate(Unpacked, Ctx, Match) :-
	% compound arguments with free variables need to be handled
	% separately because we cannot write path queries that
	% access array elements.
	set_nested_args(Unpacked,SetArgs),
	match_nested(Unpacked, Ctx, MatchNested),
	(	Match = SetArgs
	;	Match = MatchNested
	).

%%
match_nested(Unpacked, Ctx, Match) :-
	nested_args(Unpacked, NestedArgs),
	match_predicate(NestedArgs, Ctx, Match).

%%
nested_args([], []) :- !.

nested_args([[Key,Val,Is]|Xs], [[Key1,Val,[]]|Ys]) :-
	Is \== [],!,
	atomic_list_concat([Key|Is], '_', Key1),
	nested_args(Xs,Ys).

nested_args([_|Xs], Ys) :-
	nested_args(Xs,Ys).

%%
match_conditional(FieldKey, Arg, Ctx, ['$expr', ['$or', array([
			% pass through if var is not grounded
			['$eq',       array([string(ArgType),   string('var')])],
			% else perform a match
			[ArgOperator, array([string(FieldQuery), string(ArgValue)])]
		])]]) :-
	% get the variable in Arg term
	mng_strip_variable(Arg, Arg0),
	term_variables(Arg0, [ArgVar]),!,
	mongolog:var_key(ArgVar, Ctx, ArgKey),
	% get the operator
	mng_strip_operator(Arg0, Operator1, _Arg1),
	mng_operator(Operator1, ArgOperator),
	atom_concat('$',FieldKey,FieldQuery),
	atom_concat('$$',ArgKey,ArgValue),
	atom_concat(ArgValue,'.type',ArgType).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_database').

test('add_database_predicate') :-
	assert_true(mongolog_add_predicate(woman, [name], [[name]])),
	assert_true(mongolog_add_predicate(loves, [a,b], [[a],[b],[a,b]])).

test('assert(woman)') :-
	assert_true(mongolog_call(assert(woman(mia)))),
	assert_true(mongolog_call(assert(woman(jody)))).

test('woman(+)') :-
	assert_true(mongolog_call(woman(mia))),
	assert_true(mongolog_call(woman(jody))),
	assert_false(mongolog_call(woman(vincent))).

test('woman(-)') :-
	findall(X, mongolog_call(woman(X)), Xs),
	assert_unifies([_,_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(mia,Xs)),
	assert_true(memberchk(jody,Xs)).

test('retract(woman)') :-
	assert_true(mongolog_call(woman(jody))),
	assert_true(mongolog_call(retractall(woman(jody)))),
	assert_false(mongolog_call(woman(jody))).

test('assert(loves)') :-
	assert_true(mongolog_call(assert(loves(vincent,mia)))),
	assert_true(mongolog_call(assert(loves(marsellus,jody)))),
	assert_true(mongolog_call(assert(loves(pumpkin,honey_bunny)))).

test('loves(+,+)') :-
	assert_true(mongolog_call(loves(vincent,mia))),
	assert_true(mongolog_call(loves(marsellus,jody))),
	assert_false(mongolog_call(loves(mia,vincent))).

test('loves(+,-)') :-
	findall(X, mongolog_call(loves(vincent,X)), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(mia,Xs)).

test('loves(-,+)') :-
	findall(X, mongolog_call(loves(X,mia)), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(vincent,Xs)).

test('assert(shape)') :-
	assert_true(mongolog_add_predicate(shape, [name,term], [[name]])),
	assert_true(mongolog_call(assert(shape(obj1,sphere(1.0))))),
	assert_true(mongolog_call(assert(shape(obj3,sphere(2.0))))),
	assert_true(mongolog_call(assert(shape(obj2,box(1.0,2.0,3.0))))).

test('shape(+,+)') :-
	assert_true(mongolog_call(shape(obj1,sphere(1.0)))),
	assert_true(mongolog_call(shape(obj2,box(1.0,2.0,3.0)))),
	assert_false(mongolog_call(shape(obj1,cylinder(1.0)))),
	assert_false(mongolog_call(shape(obj2,sphere(1.0)))).

test('shape(+,-)') :-
	mongolog_call(shape(obj1,Term)),
	assert_equals(Term, sphere(1.0)).

test('shape(-,+)') :-
	findall(X, mongolog_call(shape(X,sphere(1.0))), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(obj1,Xs)).

test('shape(+,sphere(-))') :-
	findall(X, mongolog_call(shape(obj1,sphere(X))), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(1.0,Xs)).

test('+Cond->assert(woman);assert(woman)') :-
	assert_false(mongolog_call(woman(bar))),
	assert_true(lang_query:test_command(
		(	Num > 5
		->	assert(woman(foo))
		;	assert(woman(bar))
		),
		Num, 4.5)),
	assert_true(mongolog_call(woman(bar))),
	assert_false(mongolog_call(woman(foo))).

test('drop_database_predicate') :-
	assert_true(mongolog_drop_predicate(shape)),
	assert_true(mongolog_drop_predicate(woman)),
	assert_true(mongolog_drop_predicate(loves)).

:- end_tests('mongolog_database').
