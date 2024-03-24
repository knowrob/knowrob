:- module(mongolog_database,
		[ mongolog_get_db/3,
		  mongolog_one_db/2,
		  mongolog_uri/1,
		  mongolog_is_readonly/0,
		  mongolog_export/1,
		  mongolog_import/1,
		  mongolog_add_predicate/3,
		  mongolog_add_predicate/4,
		  mongolog_drop_predicate/1,
		  mongolog_predicate/6,
		  mongolog_predicate_document(t,-),
		  mongolog_predicate_collection(t,-,-)
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
:- use_module('client').

%% Predicates that are stored in a mongo collection
:- dynamic mongolog_predicate/6.

%% query commands
:- mongolog:add_command(assert).
:- mongolog:add_command(retractall).

%% mongolog_is_readonly is det.
%
% Only true if mongolog has a read-only connection to the database.
%
mongolog_is_readonly :-
	current_reasoner_manager(Manager),
	current_reasoner_module(ReasonerModule),
	mng_is_readonly_cpp(Manager, ReasonerModule).

%% mongolog_get_db(?DB, -Collection, +DBType) is det.
%
% Get database and collection name for type
% of data denoted by DBType identifier.
% The type identifier can be choosen freely but should
% not conflict with another collection in the database.
%
% @param DB The database name
% @param Collection The collection name
% @param DBType type identifier
%
mongolog_get_db(db(URI,DB), Collection, DBType) :-
    mongolog_uri(URI),
    mongolog_db_name(DB),
    Collection = DBType.

%% mongolog_one_db(?DB, -Collection) is det.
%
% Get a special database collection with just one empty document.
% This is used for feeding just this one document into aggregate
% pipelines.
%
% @param DB The database name
% @param Collection The collection name
%
mongolog_one_db(db(URI,DB), one) :-
    mongolog_uri(URI),
    mongolog_db_name(DB).

%%
mongolog_db_name(DB) :-
	current_reasoner_manager(Manager),
	current_reasoner_module(ReasonerModule),
	mng_db_name_cpp(Manager, ReasonerModule, DB),
	!.
mongolog_db_name(knowrob).


%% mongolog_uri(-URI) is det.
%
% Get the URI connection string
%
mongolog_uri(URI) :-
	current_reasoner_manager(Manager),
	current_reasoner_module(ReasonerModule),
	mng_uri_cpp(Manager, ReasonerModule, URI),
    !.

mongolog_uri(URI) :-
    getenv('KNOWROB_MONGO_HOST', Host),
    getenv('KNOWROB_MONGO_USER', User),
    getenv('KNOWROB_MONGO_PASS', PW),
    getenv('KNOWROB_MONGO_PORT', Port), !,
    mongolog_uri_(Host,Port,User,PW,URI).

mongolog_uri('mongodb://localhost:27017').

%%
mongolog_uri_(Host, Port, '', '', URI) :-
    !, atomic_list_concat([ 'mongodb://', Host, ':', Port ], URI).
mongolog_uri_(Host, Port, User, '', URI) :-
    !, atomic_list_concat([ 'mongodb://', User, '@', Host, ':', Port ], URI).
mongolog_uri_(Host, Port, User, PW, URI) :-
    !, atomic_list_concat([ 'mongodb://', User, ':', PW, '@', Host, ':', Port ], URI).

%% mongolog_export(+Path) is semidet.
%
% Exports the MongoDB database to local filesystem.
%
mongolog_export(Path) :-
    mongolog_db_name(DB),
    mng_dump(DB, Path).

%% mongolog_impor(+Path) is semidet.
%
% Imports the MongoDB database from local filesystem.
%
mongolog_import(Path) :-
    mongolog_db_name(DB),
    mng_restore(DB, Path).

%% mongolog_predicate(+Term, ?Arity, ?Fields, ?DstModule, ?SourceID, ?Options) is semidet.
%
%
mongolog_predicate(Term, Arity, Fields, DstModule, SourceID, Options) :-
	compound(Term),!,
	Term =.. [Functor|Args],
	length(Args, Arity),
	mongolog_predicate(Functor, Arity, Fields, DstModule, SourceID, Options).

%% mongolog_predicate_document(+Predicate, -PredicateDoc) is det.
%
%
mongolog_predicate_document(Predicate, PredicateDoc) :-
    compound(Predicate),
	mongolog_database:mongolog_predicate_zip(Predicate,
		[], Zipped, Ctx_pred, write),
	findall([Field,Val],
		(	member([Field,Arg],Zipped),
			mongolog:var_key_or_val(Arg, Ctx_pred, Val)
		),  PredicateDoc).

%% mongolog_predicate_collection(+Predicate, -DB, -Collection) is semidet.
%
%
mongolog_predicate_collection(Predicate, DB, PredicateCollection) :-
    compound(Predicate),
	Predicate =.. [Functor|_Args],
	mongolog_get_db(DB, PredicateCollection, Functor).

%%
%
% Same as mongolog_add_predicate/4 but automatically constructs
% fields using the index as a key.
%
mongolog_add_predicate(Functor/Arity, SourceID, Options) :-
    % use argument number as name
    findall(FieldName,
        (   between(1,Arity,X),
    		atom_number(FieldName,X)
        ),  Fields),
    mongolog_add_predicate(Functor, Fields, SourceID, Options).

%% mongolog_add_predicate(+Functor, +Fields, +SourceID, +Options) is semidet.
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
mongolog_add_predicate(Functor, Fields, _, _) :-
	current_reasoner_module(DstModule),
	length(Fields, Arity),
	mongolog_predicate(Functor, Arity, _, RealModule, _, _),
	(RealModule==user ; RealModule==DstModule),
	!,
	throw(permission_error(modify,database_predicate,Functor)).

mongolog_add_predicate(Functor, Fields, SourceID, Options) :-
	length(Fields, Arity),
	current_reasoner_module(DstModule),
	setup_predicate_collection(Functor, Fields, Options),
	assertz(mongolog_predicate(Functor, Arity, Fields, DstModule, SourceID, Options)),
	mongolog:add_command(Functor, DstModule).

%%
setup_predicate_collection(Functor, [FirstField|_], Options) :-
	option(indices(Indices), Options, [[FirstField], [source]]),
	setup_collection(Functor, Indices).


%% mongolog_drop_predicate(+Functor) is det.
%
% Delete all facts associated to predicate with
% given functor.
%
% @param Functor functor of the predicate
%
mongolog_drop_predicate(Functor) :-
	mongolog_get_db(DB, Collection, Functor),
	retractall(mongolog_predicate(Functor, _, _, _, _, _)),
	mng_drop(DB, Collection).

%%
mongolog:step_expand(project(Term), assert(Term)) :-
	mongolog_predicate(Term, _, _, _, _, _),!.

%%
mongolog:step_compile(assert(Term), Ctx, Pipeline, StepVars) :-
	mongolog_predicate(Term, _, _, _, _, _),!,
	mongolog_predicate_assert(Term, Ctx, Pipeline, StepVars).

mongolog:step_compile(retractall(Term), Ctx, Pipeline, StepVars) :-
	mongolog_predicate(Term, _, _, _, _, _),!,
	mongolog_predicate_retractall(Term, Ctx, Pipeline, StepVars).

mongolog:step_compile(Term, Ctx, Pipeline, StepVars) :-
	mongolog_predicate(Term, _, _, _, _, _),!,
	mongolog_predicate_call(Term, Ctx, Pipeline, StepVars).

%%
mongolog_predicate_call(Term, Ctx, Pipeline, StepVars) :-
	mongolog_predicate_zip(Term, Ctx, Zipped, Ctx_pred, read),
	option(step_vars(StepVars), Ctx_pred),
	%
	unpack_compound(Zipped, Unpacked),
	findall(InnerStep,
		match_predicate(Unpacked, Ctx_pred, InnerStep),
		InnerPipeline),
	%
	findall(Step,
		% look-up documents into 't_pred' array field
		(	lookup_predicate('t_pred', InnerPipeline, Ctx_pred, Step)
		% unwind lookup results and assign variables
		;	Step=['$unwind', string('$t_pred')]
		;	project_predicate(Unpacked, Ctx_pred, Step)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline).

%%
mongolog_predicate_retractall(Term, Ctx, Pipeline, StepVars) :-
	mongolog_predicate_zip(Term, Ctx, Zipped, Ctx_pred, write),
	option(collection(Collection), Ctx_pred),
	option(step_vars(StepVars), Ctx_pred),
	unpack_compound(Zipped, Unpacked),
	findall(InnerStep,
		(	match_predicate(Unpacked, Ctx_pred, InnerStep)
		% retractall first performs match, then only projects the id of the document
		;	project_retract(InnerStep)
		),
		InnerPipeline),
	%
	findall(Step,
		% look-up documents into 't_pred' array field
		(	lookup_predicate('t_pred', InnerPipeline, Ctx_pred, Step)
		% add removed facts to assertions list
		;	mongolog:add_assertions(string('$t_pred'), Collection, Step)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline
	).

%%
mongolog_predicate_assert(Term, Ctx, Pipeline, StepVars) :-
	mongolog_predicate_zip(Term, Ctx, Zipped, Ctx_pred, write),
	option(collection(Collection), Ctx_pred),
	option(step_vars(StepVars), Ctx_pred),
	option(source_id(SourceID), Ctx_pred, user),
	% create a document
	findall([Field,Val],
		(	member([Field,Arg],Zipped),
			mongolog:var_key_or_val(Arg, Ctx_pred, Val)
		), PredicateDoc0),
	% add source id to document
	PredicateDoc = [['source', string(SourceID)] | PredicateDoc0],
	% and add it to the list of asserted documents
	findall(Step,
		mongolog:add_assertion(PredicateDoc, Collection, Step),
		Pipeline).

%%
%
mongolog_predicate_zip(Term, Ctx, Zipped, Ctx_zipped, ReadOrWrite) :-
	% get predicate fields and options
	mongolog_predicate(Term, _, ArgFields, _, _, Options),
	% get predicate functor and arguments
	Term =.. [Functor|Args],
	% get the database collection of the predicate
	(	option(collection(Collection), Options)
	;	mongolog_get_db(_DB, Collection, Functor)
	),
	!,
	% read variable in Term
	mongolog:step_vars(Term, Ctx, StepVars0),
	(	ReadOrWrite==read -> StepVars=StepVars0
	;	mongolog:add_assertion_var(StepVars0, StepVars)
	),
	% add predicate options to compile context
	merge_options([
		step_vars(StepVars),
		collection(Collection)
	], Ctx, Ctx0),
	merge_options(Ctx0, Options, Ctx_zipped),
	% zip field names with predicate arguments
	zip(ArgFields, Args, Zipped).

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
	assert_true(mongolog_add_predicate(test_woman, [name], test, [])),
	assert_true(mongolog_add_predicate(test_loves, [a,b], test, [])).

test('assert(woman)') :-
	assert_true(mongolog_call(assert(test_woman(mia)))),
	assert_true(mongolog_call(assert(test_woman(jody)))).

test('woman(+)') :-
	assert_true(mongolog_call(test_woman(mia))),
	assert_true(mongolog_call(test_woman(jody))),
	assert_false(mongolog_call(test_woman(vincent))).

test('woman(-)') :-
	findall(X, mongolog_call(test_woman(X)), Xs),
	assert_unifies([_,_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(mia,Xs)),
	assert_true(memberchk(jody,Xs)).

test('retract(woman)') :-
	assert_true(mongolog_call(test_woman(jody))),
	assert_true(mongolog_call(retractall(test_woman(jody)))),
	assert_false(mongolog_call(test_woman(jody))).

test('assert(loves)') :-
	assert_true(mongolog_call(assert(test_loves(vincent,mia)))),
	assert_true(mongolog_call(assert(test_loves(marsellus,jody)))),
	assert_true(mongolog_call(assert(test_loves(pumpkin,honey_bunny)))).

test('loves(+,+)') :-
	assert_true(mongolog_call(test_loves(vincent,mia))),
	assert_true(mongolog_call(test_loves(marsellus,jody))),
	assert_false(mongolog_call(test_loves(mia,vincent))).

test('loves(+,-)') :-
	findall(X, mongolog_call(test_loves(vincent,X)), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(mia,Xs)).

test('loves(-,+)') :-
	findall(X, mongolog_call(test_loves(X,mia)), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(vincent,Xs)).

test('assert(shape)') :-
	assert_true(mongolog_add_predicate(test_shape, [name,term], test, [])),
	assert_true(mongolog_call(assert(test_shape(obj1,sphere(1.0))))),
	assert_true(mongolog_call(assert(test_shape(obj3,sphere(2.0))))),
	assert_true(mongolog_call(assert(test_shape(obj2,box(1.0,2.0,3.0))))).

test('shape(+,+)') :-
	assert_true(mongolog_call(test_shape(obj1,sphere(1.0)))),
	assert_true(mongolog_call(test_shape(obj2,box(1.0,2.0,3.0)))),
	assert_false(mongolog_call(test_shape(obj1,cylinder(1.0)))),
	assert_false(mongolog_call(test_shape(obj2,sphere(1.0)))).

test('shape(+,-)') :-
	mongolog_call(test_shape(obj1,Term)),
	assert_equals(Term, sphere(1.0)).

test('shape(-,+)') :-
	findall(X, mongolog_call(test_shape(X,sphere(1.0))), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(obj1,Xs)).

test('shape(+,sphere(-))') :-
	findall(X, mongolog_call(test_shape(obj1,sphere(X))), Xs),
	assert_unifies([_],Xs),
	assert_true(ground(Xs)),
	assert_true(memberchk(1.0,Xs)).

test('+Cond->assert(woman);assert(woman)') :-
	assert_false(mongolog_call(test_woman(bar))),
	assert_true(mongolog:test_call(
		(	Num > 5
		->	assert(test_woman(foo))
		;	assert(test_woman(bar))
		),
		Num, 4.5)),
	assert_true(mongolog_call(test_woman(bar))),
	assert_false(mongolog_call(test_woman(foo))).

test('drop_database_predicate') :-
	assert_true(mongolog_drop_predicate(test_shape)),
	assert_true(mongolog_drop_predicate(test_woman)),
	assert_true(mongolog_drop_predicate(test_loves)).

:- end_tests('mongolog_database').
