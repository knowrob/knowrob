:- module(mongolog_fluents,
		[ mongolog_add_fluent(+,+,+,+),
		  mongolog_add_fluent(+,+,+),
		  mongolog_drop_fluent(+)
		]).
/** <module> Fluents in mongolog programs.

Fluents are functional predicates that change their value over time.
Documents with a dedicated time field are used to represent fluents.
Any existing database collection with time index may be used as a knowledgeGraph for fluents.
At any timepoint, the document with the latest stamp before the timepoint provides fluent value.
There can only be one fluent value at a time.
However, fluents may have additional "value" arguments. For each combination of value arguments
a different value is stored.
The value of a fluent is given by the remaining "value" arguments.
Each argument may be an atomic value, or a compound term.
Fluents support assert/1 and retractall/1 for adding and removing them from the database.
All value arguments must be grounded, value arguments maybe grounded to apply additional
filtering based on the fluent value.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('option'), [ option/3 ]).
:- use_module(library('apply'), [ convlist/3 ]).
:- use_module('mongolog').
:- use_module('client').
:- use_module(library('scope')).

%% Predicates that are stored in a mongo collection
:- dynamic mongolog_fluent/4.

%% mongolog_fluent(+Term, -TimeField, -ArgFields, -Options) is semidet.
%
%
mongolog_fluent(Term, TimeField, ArgFields, Options) :-
	compound(Term),!,
	Term =.. [Functor|_],
	mongolog_fluent(Functor, TimeField, ArgFields, Options).

%% mongolog_add_fluent(+Functor, +ArgFields, +TimeField, +Options) is semidet.
%
% Register a fluent predicate that stores facts in the database.
% Functor is the functor of a n-ary predicate, and Fields is
% a n-elemental list of keys associated to the different
% arguments of the predicate.
% TimeKey is the document field where the time is stored.
% Options may have the same options as accepted by mongolog_add_predicate/3.
%
% Current limitation: there cannot be predicates with the same functor,
% but different arity.
%
% @param Functor functor of the predicate
% @param ArgFields field names of predicate arguments
% @param TimeField field name of time value
% @param Options option list
%
mongolog_add_fluent(Functor, _, _, _) :-
    % TODO: allow that the same fluent is defined in different reasoner modules.
    %   (look at database.pl handling of facts). This is needed when multiple instances
    %   of Mongolog reasoner are used.
	mongolog_fluent(Functor, _, _, _),
	!,
	throw(permission_error(modify,database_fluent,Functor)).

mongolog_add_fluent(Functor, ArgFields, TimeField, Options) :-
	setup_fluent_collection(Functor, ArgFields, TimeField, Options),
	assertz(mongolog_fluent(Functor, ArgFields, TimeField, Options)),
	current_reasoner_module(ReasonerModule),
	mongolog:add_command(Functor, ReasonerModule).

%%
setup_fluent_collection(Functor, ArgFields, TimeField, Options) :-
	option(indices(Indices), Options, auto),
	setup_fluent_collection1(Indices, Functor, ArgFields, TimeField, Options).

setup_fluent_collection1(auto, Functor, ArgFields, TimeField, Options) :-
	!,
	% create default indices:
	%	- time index
	%	- time + fluent keys index (if fluent has any keys)
	%
	convlist([X,Y]>>
		(X=(+(Key)), Y=Key),
		ArgFields, KeyFields),
	findall(Index,
		(	Index=[TimeField]
		;	(	KeyFields \== [],
				Index=[TimeField|KeyFields]
			)
		),
		Indices),
	setup_fluent_collection1(Indices, Functor, ArgFields, TimeField, Options).

setup_fluent_collection1(Indices, Functor, _, _, Options) :-
	fluent_collection(Functor, Options, Collection),
	setup_collection(Collection, Indices).


%% mongolog_add_fluent(+Functor, +ArgFields, +TimeField) is semidet.
%
% Same as mongolog_add_fluent/4 with empty options list.
%
% @param Functor functor of the predicate
% @param ArgFields field names of predicate arguments
% @param TimeField field name of time value
%
mongolog_add_fluent(Functor, ArgFields, TimeField) :-
	mongolog_add_fluent(Functor, ArgFields, TimeField, []).


%% mongolog_drop_fluent(+Functor) is det.
%
% Delete all facts associated to predicate with
% given functor.
%
% @param Functor functor of the predicate
%
mongolog_drop_fluent(Functor) :-
	mongolog_get_db(DB, Collection, Functor),
	mng_drop(DB, Collection).

%%
mongolog:step_expand(project(Term), assert(Term)) :-
	mongolog_fluent(Term, _, _, _),!.

%%
%
mongolog:step_compile(assert(Term), Ctx, Pipeline, StepVars) :-
	mongolog_fluent(Term, _, _, _),!,
	mongolog_fluent_assert(Term, Ctx, Pipeline, StepVars).

mongolog:step_compile(retractall(Term), Ctx, Pipeline, StepVars) :-
	mongolog_fluent(Term, _, _, _),!,
	mongolog_fluent_retractall(Term, Ctx, Pipeline, StepVars).

mongolog:step_compile(Term, Ctx, Pipeline, StepVars) :-
	mongolog_fluent(Term, _, _, _),!,
	mongolog_fluent_call(Term, Ctx, Pipeline, StepVars).

%%
%
mongolog_fluent_call(Term, Ctx, Pipeline, StepVars) :-
	fluent_zip(Term, Ctx,
		ZippedKeyFields,
		ZippedValueFields,
		TimeField, Ctx_fluent, read),
	option(step_vars(StepVars), Ctx_fluent),
	% get since+until time
	option(query_scope(Scope), Ctx_fluent),
	mongolog_time_scope(Scope, Since_typed, Until_typed),
	% FIXME: below makes it impossible to use Since variable
	%        inferred in the query.
	mng_strip_type(Since_typed, _, Since),
	mng_strip_type(Until_typed, _, Until),
	% create aggregate pipeline
	mongolog_database:unpack_compound(ZippedKeyFields,   KeyFields),
	mongolog_database:unpack_compound(ZippedValueFields, ValueFields),
	findall(Step,
		% look-up documents into 't_pred' array field
		(	fluent_lookup(KeyFields, ValueFields, TimeField, Since, Until, Ctx_fluent, Step)
		% unwind lookup results and assign variables
		;	Step=['$unwind', string('$t_pred')]
		% get fluent time
		;	(	atomic_list_concat(['$t_pred.',TimeField], '', TimeKey0),
				Step=['$set', ['v_fluent_time', string(TimeKey0)]]
			)
		% need to perform additional lookup to get the next value after During
		;	fluent_lookup_next('t_next', KeyFields, ValueFields, TimeField, Ctx_fluent, Step)
		% update v_scope
		;	fluent_fact_scope(TimeField, Step)
		;	Step=['$unset', string('t_next')]
		;	Step=['$unset', string('v_fluent_time')]
		% finally project predicate arguments
		;	mongolog_database:project_predicate(ValueFields, Ctx_fluent, Step)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline).

%%
%
mongolog_fluent_retractall(Term, Ctx, Pipeline, StepVars) :-
	fluent_zip(Term, Ctx,
		ZippedKeyFields,
		ZippedValueFields,
		TimeField, Ctx_fluent, write),
	option(step_vars(StepVars), Ctx_fluent),
	option(collection(Collection), Ctx_fluent),
	% get since+until time
	option(query_scope(Scope), Ctx_fluent),
	mongolog_time_scope(Scope, Since_typed, Until_typed),
	% FIXME: below makes it impossible to use Since variable
	%        inferred in the query.
	mng_strip_type(Since_typed, _, Since),
	mng_strip_type(Until_typed, _, Until),
	% create aggregate pipeline
	mongolog_database:unpack_compound(ZippedKeyFields,   UnpackedKeys),
	mongolog_database:unpack_compound(ZippedValueFields, UnpackedValues),
	findall(Step,
		% look-up documents into 't_pred' array field
		(	fluent_lookup(UnpackedKeys, UnpackedValues,
				TimeField, Since, Until, [retract|Ctx_fluent], Step)
		% finally push documents to deletion list
		;	mongolog:add_assertions(string('$t_pred'), Collection, Step)
		;	Step=['$unset', string('t_pred')]
		),
		Pipeline).

%%
%
mongolog_fluent_assert(Term, Ctx, Pipeline, StepVars) :-
	fluent_zip(Term, Ctx,
		ZippedKeyFields,
		ZippedValueFields,
		TimeField, Ctx_fluent, write),
	option(step_vars(StepVars), Ctx_fluent),
	option(collection(Collection), Ctx_fluent),
	% add since time to Zipped list
	option(query_scope(Scope), Ctx_fluent),
	mongolog_time_scope(Scope, Since_typed, _),
	% FIXME: below makes it impossible to use Since variable
	%        inferred in the query.
	mng_strip_type(Since_typed, _, Since),
	append([[TimeField,time(Since)]|ZippedKeyFields], ZippedValueFields, Zipped),
	% create a document
	findall([Field,Val],
		(	member([Field,Arg],Zipped),
			mongolog:var_key_or_val(Arg, Ctx, Val)
		),
		PredicateDoc),
	% make sure we have a nested doc since keys with a '.' are not allowed here!
	mng_unflatten(PredicateDoc, NestedDoc),
	% and add it to the list of asserted documents
	findall(Step,
		mongolog:add_assertion(NestedDoc, Collection, Step),
		Pipeline).

%%
%
fluent_zip(Term, Ctx, ZippedKeys, ZippedValues, TimeKey, Ctx_zipped, ReadOrWrite) :-
	mongolog_fluent(Term, ArgFields, TimeKey, Options),
	% get predicate functor and arguments
	Term =.. [Functor|Args],
	% get the database collection of the predicate
	fluent_collection(Functor, Options, Collection),
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
	zip(ArgFields, Args, Zipped),
	% group into key and value fields
	convlist([X,Y]>>
		(X=[+(Key),Arg], Y=[Key,Arg]),
		Zipped, ZippedKeys),
	convlist([X,Y]>>
		(X=[-(Key),Arg], Y=[Key,Arg]),
		Zipped, ZippedValues).

%% $lookup
%
fluent_lookup(UnpackedKeys, UnpackedValues, TimeKey, Since, _, Ctx, Step) :-
	% lookup the latest document before Since
	findall(InnerStep,
		(	mongolog_database:match_predicate([
				[TimeKey,=<(time(Since)),[]]|
				UnpackedKeys
			], Ctx, InnerStep)
		;	InnerStep=['$sort',[TimeKey,int(-1)]]
		;	InnerStep=['$limit',int(1)]
		% match fluent values given in the query
		% NOTE: it is important this happens _after_ $sort and $limit
		;	mongolog_database:match_predicate(UnpackedValues, Ctx, InnerStep)
		;	( option(retract,Ctx), mongolog_database:project_retract(InnerStep) )
		),
		InnerPipeline),
	mongolog_database:lookup_predicate('t_pred', InnerPipeline, Ctx, Step).

fluent_lookup(UnpackedKeys, UnpackedValues, TimeKey, Since, Until, Ctx, Step) :-
	% lookup all documents between Since and Until
	Since \== Until,
	append(UnpackedKeys, UnpackedValues, Unpacked),
	findall(InnerStep,
		(	mongolog_database:match_predicate([
				[TimeKey,=<(time(Until)),[]],
				[TimeKey,>(time(Since)),[]]|
				Unpacked
			], Ctx, InnerStep)
		;	InnerStep=['$sort',[TimeKey,int(-1)]]
		;	( option(retract,Ctx), mongolog_database:project_retract(InnerStep) )
		),
		InnerPipeline),
	mongolog_database:lookup_predicate('t_pred_1', InnerPipeline, Ctx, Step).

fluent_lookup(_, _, _, Since, Until, _,
		['$set', ['t_pred', ['$concatArrays', array([
			string('$t_pred_1'),
			string('$t_pred')
		])]]]) :-
	Since \== Until.

fluent_lookup(_, _, _, Since, Until, _,
		['$unset', string('t_pred_1')]) :-
	Since \== Until.

%%
fluent_lookup_next(ArrayField, UnpackedKeys, UnpackedValues, TimeKey, Ctx, Step) :-
	% lookup the earliest document after Until
	atom_concat('$',TimeKey,TimeKey0),
	% add 'v_fluent_time' to step_vars and outer_vars list
	FluentVar=['v_fluent_time',VarFluentTime],
	select_option(step_vars(StepVars),   Ctx,      Ctx_tmp0),
	select_option(outer_vars(OuterVars), Ctx_tmp0, Ctx_tmp1),
	StepVars_inner  = [FluentVar|StepVars],
	OuterVars_inner = [FluentVar|OuterVars],
	Ctx_inner=[
		step_vars(StepVars_inner),
		outer_vars(OuterVars_inner)
	| Ctx_tmp1],
	% lookup earliest document after Until
	findall(InnerStep,
		(	mongolog_database:match_predicate([
				[TimeKey,>(VarFluentTime),[]]|
				UnpackedKeys
			], Ctx_inner, InnerStep)
		;	InnerStep=['$sort',[TimeKey,int(1)]]
		;	InnerStep=['$limit',int(1)]
		% match fluent values given in the query
		% NOTE: it is important this happens _after_ $sort and $limit
		;	mongolog_database:match_predicate(UnpackedValues, Ctx, InnerStep)
		% convert ISODate to unix timestamp
		;	InnerStep=['$set', [TimeKey, ['$divide', array([
				['$toDecimal', string(TimeKey0)],
				double(1000.0)
			])]]]
		),
		InnerPipeline),
	mongolog_database:lookup_predicate(ArrayField, InnerPipeline, Ctx_inner, Step).

%%
fluent_fact_scope(TimeKey, Step) :-
	atomic_list_concat(['$$this.',TimeKey], '', TimeKey0),
	% update v_scope.time.since
	% fluent time is stored as ISODate in 'v_fluent_time' field
	(	Step=['$set', ['v_scope.time.since', ['$max', array([
			string('$v_scope.time.since'),
			['$divide', array([
				['$toDecimal', string('$v_fluent_time')],
				double(1000.0)
			])]
		])]]]
	% update v_scope.time.until
	% next fluent time is stored as ISODate in first element of 't_next' array field
	;	Step=['$set', ['v_scope.time.until', ['$cond', array([
			['$ne', array([string('$t_next'), array([])])],
			['$min', array([
				string('$v_scope.time.until'),
				['$arrayElemAt', array([['$map', [
					[input, string('$t_next')],
					[in, string(TimeKey0)]
				]], int(0)])]
			])],
			string('$v_scope.time.until')
		])]]]
	% make sure scope is non-empty
	;	mongolog_scope_is_valid(Step)
	).

%%
fluent_collection(Functor, Options, Collection) :-
	(	option(collection(Collection), Options)
	;	mongolog_get_db(_DB, Collection, Functor)
	).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_fluents').

test('mongolog_add_fluent') :-
	assert_true(mongolog_add_fluent(
		test_fluent,   % fluent functor name
		[-value],      % fluent argument fields
		time           % fluent time field
	)).

test('mongolog_assert(test_fluent(+)') :-
	assert_true(mongolog_call(assert(test_fluent(4.0)))).

test('mongolog_call(test_fluent(?))') :-
	assert_true(mongolog_call(test_fluent(4.0))),
	assert_false(mongolog_call(test_fluent(6.0))),
	(	mongolog_call(test_fluent(X))
	->	assert_equals(X,4.0)
	;	true
	).

test('mongolog_update(test_fluent(6.0))') :-
	assert_true(mongolog_call(test_fluent(4.0))),
	assert_true(mongolog_call(assert(test_fluent(6.0)))),
	assert_true(mongolog_call(test_fluent(6.0))),
	(	mongolog_call(test_fluent(X))
	->	assert_equals(X,6.0)
	;	true
	),
	assert_false(mongolog_call(test_fluent(4.0))).

test('mongolog_retract(test_fluent(6.0))') :-
	assert_true(mongolog_call(test_fluent(6.0))),
	assert_true(mongolog_call(retractall(test_fluent(6.0)))),
	assert_true(mongolog_call(test_fluent(4.0))),
	assert_false(mongolog_call(test_fluent(6.0))).

test('mongolog_drop(test_fluent)') :-
	assert_true(mongolog_drop_fluent(test_fluent)).

:- end_tests('mongolog_fluents').
