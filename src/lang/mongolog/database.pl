:- module(lang_database,
		[ add_database_predicate/3,
		  drop_database_predicate/1
		]).

:- use_module(library('lang/compiler')).

:- dynamic database_predicate/2.

%% query commands
:- query_compiler:add_command(assert).
%:- query_compiler:add_command(retract).

add_database_predicate(Functor, _, _) :-
	database_predicate(Functor, _),
	!,
	throw(permission_error(modify,database_predicate,Functor)).

add_database_predicate(Functor, Fields, Options) :-
	setup_predicate_collection(Functor, Fields, Options),
	assertz(database_predicate(Functor, Fields)),
	query_compiler:add_command(Functor).

%%
drop_database_predicate(Functor) :-
	mng_get_db(DB, Collection, Functor),
	mng_drop(DB, Collection).

%%
query_compiler:step_compile(Term, Ctx, Pipeline, StepVars) :-
	compound(Term),
	Term =.. [Functor|_],
	database_predicate(Functor, ArgFields),
	!,
	query_compiler:step_vars(Term, Ctx, StepVars0),
	(	option(mode(ask), Ctx)
	->	StepVars=StepVars0
	;	StepVars=[['g_assertions',_]|StepVars0]
	),
	merge_options([step_vars(StepVars)], Ctx, Ctx0),
	(	option(mode(ask), Ctx)
	->	query_predicate(Term, ArgFields, Ctx0, Pipeline, StepVars)
	;	assert_predicate(Term, ArgFields, Ctx0, Pipeline)
	).

query_compiler:step_compile(assert(Term), Ctx, Pipeline, StepVars) :-
	merge_options([mode(tell)], Ctx, Ctx0),
	query_compiler:step_compile(Term, Ctx0, Pipeline, StepVars).
	
%%
query_predicate(Term, ArgFields, Ctx, Pipeline, StepVars) :-
	Term =.. [Functor|Args],
	mng_get_db(_DB, Collection, Functor),
	zip(ArgFields, Args, Zipped),
	unpack_compound(Zipped, Unpacked),
	%
	query_compiler:lookup_let_doc(StepVars, LetDoc),
	merge_options([disj_vars(StepVars)], Ctx, Ctx0),
	match_predicate(Unpacked, Ctx0, Match),
	%
	findall(Step,
		% look-up documents into 't_pred' array field
		(	Step=['$lookup', [
				['from', string(Collection)],
				['as', string('t_pred')],
				['let', LetDoc],
				['pipeline', array([Match])]
			]]
		% unwind lookup results and assign variables
		;	Step=['$unwind', string('$t_pred')]
		;	(	member([FieldPath,Var], Unpacked),
				query_compiler:var_key(Var,Ctx0,VarKey),
				atom_concat('$t_pred.', FieldPath, FieldQuery),
				Step=['$set', [VarKey, string(FieldQuery)]]
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
			query_compiler:var_key_or_val(Arg, Ctx, Val)
		),
		PredicateDoc),
	% and add it to the list of asserted documents
	findall(Step,
		query_compiler:add_assertion(PredicateDoc, Collection, Step),
		Pipeline).

%%
unpack_compound([], []) :- !.

unpack_compound([X|Xs], Unpacked) :-
	unpack_compound1(X, Unpacked0),
	unpack_compound(Xs, Unpacked1),
	append(Unpacked0, Unpacked1, Unpacked).

unpack_compound1([Field,Arg], [[Field, Arg]]) :-
	(ground(Arg);var(Arg)),!.

unpack_compound1([Field,Term], [[FunctorField,Functor]|ArgsUnpacked]) :-
	compound(Term),!,
	% compound term with free variables
	Term =.. [Functor|Args],
	atom_concat(Field,'.value.functor',FunctorField),
	arg_fields_(Field, Args, 0, ArgFields),
	unpack_compound(ArgFields, ArgsUnpacked).

%%
arg_fields_(_, [], _, []) :- !.
arg_fields_(Field, [X|Xs], Index, [[ArgField,X]|Rest]) :-
	atomic_list_concat([Field,'value','args',Index], '.', ArgField),
	Index1 is Index+1,
	arg_fields_(Field, Xs, Index1, Rest).

		 /*******************************
		 *    	 	 $match   			*
		 *******************************/

%%
match_predicate(Unpacked, Ctx, Match) :-
	findall(MatchQuery,
		% first match what is grounded compile-time
		(	(	findall([DocKey, ValueQuery],
					(	member([DocKey,Value], Unpacked),
						mng_query_value(Value, ValueQuery)
					),
					MatchQuery),
				MatchQuery \== []
			)
		% next match variables grounded in call context
		;	(	member([DocKey,Var], Unpacked),
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
	query_compiler:var_key(ArgVar, Ctx, ArgKey),
	% get the operator
	mng_strip_operator(Arg0, Operator1, _Arg1),
	mng_operator(Operator1, ArgOperator),
	atom_concat('$',FieldKey,FieldQuery),
	atom_concat('$$',ArgKey,ArgValue),
	atom_concat(ArgValue,'.type',ArgType).

		 /*******************************
		 *    	  SEARCH INDICES   		*
		 *******************************/

%%
setup_predicate_collection(Functor, [FirstField|_], Options) :-
	(	option(indices(Indices), Options)
	->	setup_collection(Functor, Indices)
	;	setup_collection(Functor, [[FirstField]])
	).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('lang_database').

test('add_database_predicate') :-
	assert_true(add_database_predicate(woman, [name], [[name]])),
	assert_true(add_database_predicate(loves, [a,b], [[a],[b],[a,b]])).

test('assert(woman)') :-
	assert_true(ask(assert(woman(mia)))),
	assert_true(ask(assert(woman(jody)))).

test('woman(+)') :-
	assert_true(ask(woman(mia))),
	assert_true(ask(woman(jody))),
	assert_false(ask(woman(vincent))).

test('woman(-)') :-
	findall(X, ask(woman(X)), Xs),
	assert_unifies([_,_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(mia,Xs)),
	assert_true(memberchk(jody,Xs)).

test('assert(loves)') :-
	assert_true(ask(assert(loves(vincent,mia)))),
	assert_true(ask(assert(loves(marsellus,jody)))),
	assert_true(ask(assert(loves(pumpkin,honey_bunny)))).

test('loves(+,+)') :-
	assert_true(ask(loves(vincent,mia))),
	assert_true(ask(loves(marsellus,jody))),
	assert_false(ask(loves(mia,vincent))).

test('loves(+,-)') :-
	findall(X, ask(loves(vincent,X)), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(mia,Xs)).

test('loves(-,+)') :-
	findall(X, ask(loves(X,mia)), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(vincent,Xs)).

test('assert(shape)') :-
	assert_true(add_database_predicate(shape, [name,term], [[name]])),
	assert_true(ask(assert(shape(obj1,sphere(1.0))))),
	assert_true(ask(assert(shape(obj3,sphere(2.0))))),
	assert_true(ask(assert(shape(obj2,box(1.0,2.0,3.0))))).

test('shape(+,+)') :-
	assert_true(ask(shape(obj1,sphere(1.0)))),
	assert_true(ask(shape(obj2,box(1.0,2.0,3.0)))),
	assert_false(ask(shape(obj1,cylinder(1.0)))),
	assert_false(ask(shape(obj2,sphere(1.0)))).

test('shape(+,-)') :-
	ask(shape(obj1,Term)),
	assert_equals(Term, sphere(1.0)).

test('shape(-,+)') :-
	findall(X, ask(shape(X,sphere(1.0))), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(obj1,Xs)).

test('shape(+,sphere(-))') :-
	findall(X, ask(shape(obj1,sphere(X))), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(1.0,Xs)).

test('+Cond->assert(woman);assert(woman)') :-
	assert_false(ask(woman(bar))),
	assert_true(lang_query:test_command(
		(	Num > 5
		->	assert(woman(foo))
		;	assert(woman(bar))
		),
		Num, 4.5)),
	assert_true(ask(woman(bar))),
	assert_false(ask(woman(foo))).

test('drop_database_predicate') :-
	assert_true(drop_database_predicate(shape)),
	assert_true(drop_database_predicate(woman)),
	assert_true(drop_database_predicate(loves)).

:- end_tests('lang_database').
