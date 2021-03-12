:- module(mongolog_database,
		[]).
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

%% query commands
:- mongolog:add_command(assert).
:- mongolog:add_command(retractall).

%%
is_database_predicate(Term, ArgFields) :-
	compound(Term),
	Term =.. [Functor|_],
	mongolog:database_predicate(Functor, ArgFields).

%%
mongolog:step_compile(Term, Ctx, Pipeline, StepVars) :-
	is_database_predicate(Term, ArgFields),
	!,
	mongolog:step_vars(Term, Ctx, StepVars0),
	mongolog:add_assertion_var(StepVars0, Ctx, StepVars),
	merge_options([step_vars(StepVars)], Ctx, Ctx0),
	(	option(mode(ask), Ctx)
	->	query_predicate(Term, ArgFields, Ctx0, Pipeline, StepVars)
	;	assert_predicate(Term, ArgFields, Ctx0, Pipeline)
	).

mongolog:step_compile(assert(Term), Ctx, Pipeline, StepVars) :-
%	is_database_predicate(Term,_),
	merge_options([mode(tell)], Ctx, Ctx0),
	mongolog:step_compile(Term, Ctx0, Pipeline, StepVars).

mongolog:step_compile(retractall(Term), Ctx, Pipeline, StepVars) :-
%	is_database_predicate(Term,_),
	merge_options([mode(ask),retractall], Ctx, Ctx0),
	mongolog:step_compile(Term, Ctx0, Pipeline, StepVars).
	
%%
query_predicate(Term, ArgFields, Ctx, Pipeline, StepVars) :-
	Term =.. [Functor|Args],
	mng_get_db(_DB, Collection, Functor),
	zip(ArgFields, Args, Zipped),
	unpack_compound(Zipped, Unpacked),
	%
	mongolog:lookup_let_doc(StepVars, LetDoc),
	match_predicate(Unpacked, Ctx, Match),
	findall(InnerStep,
		(	InnerStep=Match
		% compound arguments with free variables need to be handled
		% separately because we cannot write path queries that
		% access array elements.
		;	(	set_nested_args(Unpacked,SetArgs),
				match_nested(Unpacked, Ctx, MatchNested),
				(	InnerStep = SetArgs
				;	InnerStep = MatchNested
				)
			)
		% retractall first performs match, then only projects the id of the document
		;	(	option(retractall, Ctx),
				(	InnerStep=['$project',[['_id',int(1)]]]
				;	InnerStep=['$set',['delete',bool(true)]]
				)
			)
		),
		InnerPipeline),
	%
	findall(Step,
		% look-up documents into 't_pred' array field
		(	Step=['$lookup', [
				['from', string(Collection)],
				['as', string('t_pred')],
				['let', LetDoc],
				['pipeline', array(InnerPipeline)]
			]]
		% unwind lookup results and assign variables
		;	(	\+ option(retractall, Ctx),
				(	Step=['$unwind', string('$t_pred')]
				;	(	member([FieldPath0,Var,Is], Unpacked),
						mongolog:var_key(Var,Ctx,VarKey),
						atomic_list_concat([FieldPath0|Is],'_',FieldPath),
						atom_concat('$t_pred.', FieldPath, FieldQuery),
						Step=['$set', [VarKey, string(FieldQuery)]]
					)
				)
			)
		% add removed facts to assertions list
		;	(	option(retractall, Ctx),
				mongolog:add_assertions(string('$t_pred'), Collection, Step)
			)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline
	).

%%
assert_predicate(Term, ArgFields, Ctx, Pipeline) :-
	Term =.. [Functor|Args],
	mng_get_db(_DB, Collection, Functor),
	zip(ArgFields, Args, Zipped),
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

		 /*******************************
		 *    	 	 $match   			*
		 *******************************/

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
