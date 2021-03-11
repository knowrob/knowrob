:- module(mongolog,
	[ mongolog_assert(t),
	  mongolog_query(t,+,-,+),
	  mongolog_tell(t,+,+),
	  mongolog_expand(t,-,+),
	  mongolog_compile(t,-,+)
	]).
/** <module> Compiling goals into aggregation pipelines.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1, rdf_global_term/2 ]).
:- use_module(library('db/mongo/client')).

%% Stores list of terminal terms for each clause. 
:- dynamic query/4.
%% set of registered query commands.
:- dynamic step_command/1.
%% optionally implemented by query commands.
:- multifile step_expand/3.
%% implemented by query commands to compile query documents
:- multifile step_compile/3, step_compile/4.

:- rdf_meta(step_compile(t,t,t)).
:- rdf_meta(step_compile(t,t,t,-)).

%% add_command(+Command) is det.
%
% register a command that can be used in KnowRob
% language expressions and which is implemented
% in a mongo query.
% NOTE: to implement a command several multifile predicates in
% mongolog must be implemented by a command. 
%
% @param Command a command term.
%
add_command(Command) :-
	assertz(step_command(Command)).


%% mongolog_query(+Goal, +QueryScope, -FactScope, +Options) is nondet.
%
% Call Goal in ask mode.
% Facts may be scoped, i.e., they may not hold universally.
% QueryScope determines scope space that must be met by
% all yielded facts, and FactScope is the actual scope of a yielded fact.
%
% @param Goal A compound term expanding into an aggregation pipeline
% @param QueryScope Scope space
% @param FactScope Actual scope
% @param Options Additional options
%
mongolog_query(Statement, QScope, FScope, Options) :-
	query_(Statement, [scope(QScope)|Options], FScope, ask).

%% mongolog_tell(+Goal, +FactScope, +Options) is semidet.
%
% Call Goal in tell mode.
% Facts may be scoped, i.e., they may not hold universally.
% FactScope determines the scope of all facts inherited by Goal.
%
% @param Goal A compound term expanding into an aggregation pipeline
% @param FactScope Scope of facts
% @param Options Additional options
%
mongolog_tell(Statement, FScope, Options) :-
	query_(Statement, [scope(FScope)|Options], _, tell).

%%
query_(Goal, Context, FScope, Mode) :-
	% get the pipeline document
	mongolog_compile(Goal, pipeline(Doc,Vars), [mode(Mode)|Context]),
	% run the pipeline
	query_1(Doc, Vars, FScope, Mode).

query_1(Pipeline, Vars, FScope, Mode) :-
	% get DB for cursor creation. use collection with just a
	% single document as starting point.
	mng_one_db(DB, Coll),
	% run the query
	setup_call_cleanup(
		% setup: create a query cursor
		mng_cursor_create(DB, Coll, Cursor),
		% call: find matching document
		(	mng_cursor_aggregate(Cursor, ['pipeline',array(Pipeline)]),
			query_2(Mode, Cursor, Vars, FScope)
		),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
query_2(Mode, Cursor, Vars, FScope) :-
	mng_cursor_materialize(Cursor, Result),
	(	Mode==ask
	->	unify_(Result, Vars, FScope)
	;	true
	),
	assert_documents(Result).

%%
assert_documents(Result) :-
	mng_get_dict('g_assertions', Result, array(Assertions)),
	once((setof(X,
		(	member(A,Assertions),
			member(collection-string(X),A)
		),
		Collections
	);Collections=[])),
	assert_documents0(Collections, Assertions).

assert_documents0([],_) :- !.
assert_documents0([Coll|Next], Assertions) :-
	findall(Doc,
		(	member(A,Assertions),
			memberchk(collection-string(Coll),A),
			memberchk(documents-array(Docs),A),
			member(Doc,Docs)
		),
		Docs),
	assert_documents1(array(Docs), Coll),
	assert_documents0(Next, Assertions).

assert_documents1(array([]), _) :- !.
assert_documents1([], _) :- !.
assert_documents1(array(Docs), Key) :-
	% NOTE: the mongo client currently return documents as pairs A-B instead of
	%       [A,B] which is required as input.
	% TODO: make the client return docs in a format that it accepts as input.
	maplist(format_doc, Docs, Docs0),
	maplist(bulk_operation, Docs0, BulkOperations),
	mng_get_db(DB, Coll, Key),
	mng_bulk_write(DB, Coll, BulkOperations).

%%
format_doc([], []) :- !.
format_doc([X-Y0|Rest0], [[X,Y1]|Rest1]) :-
	!,
	format_doc(Y0,Y1),
	format_doc(Rest0,Rest1).
format_doc(X, X).

%%
bulk_operation(Doc, remove([['_id',id(ID)]])) :-
	memberchk(['delete',bool(Val)],Doc),!,
	memberchk(['_id',id(ID)],Doc),
	once((Val==true ; Val==1)).

bulk_operation(Doc0, update(Selector,['$set', Update])) :-
	select(['_id',id(ID)],Doc0,Update),!,
	Selector=[['_id',id(ID)]].

bulk_operation(Doc, insert(Doc)).

%%
% read accumulated fact scope and
% unify variables.
%
unify_(Result, Vars, FScope) :-
	mng_get_dict('v_scope', Result, FScope),
	unify_0(Result, Vars, Vars).

unify_0(_, _, []) :- !.
unify_0(Doc, Vars, [[VarKey, Term]|Xs]) :-
	% it is ignored here if some variables referred
	% to are not properly grounded.
	% this can happen e.g. in expressions such as (A=a;B=b)
	% where the first solution has grounded A but not B.
	(	ground(Term)
	->	once(unify_grounded(Doc, [VarKey, Term]))
	;	ignore(unify_1(Doc, Vars, [VarKey, Term]))
	),
	unify_0(Doc, Vars, Xs).

unify_grounded(Doc, [VarKey, Term]) :-
	% variable was unified in pragma command
	% make sure it did not get another grounding in the query
	mng_get_dict(VarKey, Doc, TypedValue),
	mng_strip_type(TypedValue, _, Value),
	(	Term=Value;
		(	number(Term),
			number(Value),
			Term=:=Value
		)
	).

unify_1(_, _, ['_id', _]).

unify_1(Doc, Vars, [VarKey, Val]) :-
	mng_get_dict(VarKey, Doc, TypedValue),
	unify_2(TypedValue, Vars, Val).

%%
unify_2(array(In), Vars, Out) :-
	% a variable was instantiated to a list
	!,
	unify_array(In, Vars, Out).

unify_2([K-V|Rest], Vars, Out) :-
	!,
	dict_pairs(Dict,_,[K-V|Rest]),
	unify_2(Dict, Vars, Out).

unify_2(_{
		type: string(var),
		value: string(VarKey)
	}, Vars, Out) :-
	% a variable was not instantiated
	!,
	memberchk([VarKey, VarVal], Vars),
	Out = VarVal. 

unify_2(_{
		type: string(compound),
		value: Val
	}, Vars, Out) :-
	% a variable was instantiated to a compound term
	!,
	unify_2(Val, Vars, Out).

unify_2(_{
		functor: string(Functor),
		args: Args
	}, Vars, Out) :-
	!,
	unify_2(Args, Vars, Args0),
	Out =.. [Functor|Args0].

unify_2(TypedValue, _, Value) :-
	% a variable was instantiated to an atomic value
	mng_strip_type(TypedValue, _, Value).

%%
unify_array([], _, []) :- !.
unify_array([X|Xs], Vars, [Y|Ys]) :-
	unify_2(X, Vars, Y),
	unify_array(Xs, Vars, Ys).

%% mongolog_assert(+Term) is semidet.
%
% Assert a `mongolog` rule.
% The rule is internally translated into a sequence
% of commands that can be executed in an aggregation pipeline.
% Term can be either a of the form `Head ?> Body` (ask rule) or `Head +> Body` (tell rule).
% Any non-terminal predicate in Body must have a previously asserted
% mongolog rule it can expand into.
% After being asserted, the Head predicate can be referred to in
% calls of mongolog_query/4.
%
% @param Term A mongolog rule.
%
mongolog_assert((?>(Head,Body))) :-
	query_assert1(Head, Body, ask).

mongolog_assert((+>(Head,Body))) :-
	query_assert1(Head, Body, tell).

query_assert1(Head, Body, Context) :-
	%% get the functor of the predicate
	Head =.. [Functor|Args],
	%% expand goals into terminal symbols
	(	mongolog_expand(Body, Expanded, Context) -> true
	;	log_error_and_fail(lang(assertion_failed(Body), Functor))
	),
	%% handle instantiated arguments
	% FIXME: BUG: this might create problems in cases expanded arg is nongrounded term.
	%             then to instantiate the args implicit instantiation might be needed.
	%             A solution could be adding a =/2 call _at the end_ to avoid the need
	%             for implicit instantiation.
	(	expand_arguments(Args, ExpandedArgs, ArgsGoal)
	->	(Expanded0=[ArgsGoal|Expanded], Args0=ExpandedArgs)
	;	(Expanded0=Expanded, Args0=Args)
	),
	%% store expanded query
	assertz(query(Functor, Args0, Expanded0, Context)).

%%
expand_arguments(Args, Expanded, pragma(=(Values,Vars))) :-
	expand_arguments1(Args, Expanded, Pairs),
	Pairs \= [],
	pairs_keys_values(Pairs, Values, Vars).
	

expand_arguments1([], [], []) :- !.
expand_arguments1([X|Xs], [X|Ys], Zs) :-
	var(X),!,
	expand_arguments1(Xs, Ys, Zs).
expand_arguments1([X|Xs], [Y|Ys], [X-Y|Zs]) :-
	expand_arguments1(Xs, Ys, Zs).

%% mongolog_expand(+Term, -Expanded, +Mode) is det.
%
% Translate a goal into a sequence of terminal commands.
% Terminal commands are the core predicates supported by `mongolog`
% such as arithmetic and comparison predicates.
% Rules, on the other hand, are "flattened" during term expansion,
% and translated to a sequence of these terminal commands.
%
% @param Term A compound term, or a list of terms.
% @param Expanded Sequence of terminal commands
% @param Mode 'ask' or 'tell'
%
mongolog_expand(Goal, Goal, _) :-
	% goals maybe not known during expansion, i.e. in case of
	% higher-level predicates receiving a goal as an argument.
	% these var goals need to be expanded compile-time
	% (call-time is not possible)
	var(Goal), !.

mongolog_expand(Goal, Expanded, Mode) :-
	% NOTE: do not use is_list/1 here, it cannot handle list that have not
	%       been completely resolved as in `[a|_]`.
	%       Here we check just the head of the list.
	\+ has_list_head(Goal), !,
	comma_list(Goal, Terms),
	mongolog_expand(Terms, Expanded, Mode).

mongolog_expand(Terms, Expanded, Mode) :-
	has_list_head(Terms), !,
	catch(
		expand_term_0(Terms, Expanded0, Mode),
		Exc,
		log_error_and_fail(lang(Exc, Terms))
	),
	comma_list(Buf,Expanded0),
	comma_list(Buf,Expanded1),
	% Handle cut after term expansion.
	% It is important that this is done _after_ expansion because
	% the cut within the call would yield an infinite recursion
	% otherwhise.
	% TODO: it is not so nice doing it here. would be better if it could be
	%       done in control.pl where cut operator is implemented but current
	%       interfaces don't allow to do the following operation in control.pl.
	%		(without special handling of ',' it could be done, I think)
	expand_cut(Expanded1, Expanded2),
	%%
	(	Expanded2=[One] -> Expanded=One
	;	Expanded=Expanded2
	).

%%
expand_term_0([], [], _Mode) :- !.
expand_term_0([X|Xs], [X_expanded|Xs_expanded], Mode) :-
	once(expand_term_1(X, X_expanded, Mode)),
	% could be that expand-time the list is not fully resolved
	(	var(Xs) -> Xs_expanded=Xs
	;	expand_term_0(Xs, Xs_expanded, Mode)
	).

expand_term_1(Goal, Expanded, Mode) :-
	% TODO: seems nested terms sometimes not properly flattened, how does it happen?
	is_list(Goal),!,
	expand_term_0(Goal, Expanded, Mode).

expand_term_1(Goal, Expanded, Mode) :-
	Goal =.. [Functor|_Args],
	step_command(Functor),
	% allow the goal to recursively expand
	(	step_expand(Goal, Expanded, Mode) -> true
	;	Expanded = Goal
	).

expand_term_1(Goal, Expanded, Mode) :-
	% find all asserted rules matching the functor and args
	Goal =.. [Functor|Args],
	% NOTE: do not use findall here because findall would not preserve
	%       variables in Terminals
	% NOTE: Args in query only contain vars, for instantiated vars in rule
	%       heads, pragma/1 calls are generated in Terminals (i.e. the body of the rule).
	(	bagof(Terminals,
			query(Functor, Args, Terminals, Mode),
			TerminalsList)
	->	true
	% handle the case that a predicate is referred to that wasn't
	% asserted before
	;	throw(expansion_failed(Goal,Mode))
	),
	% wrap different clauses into ';'
	semicolon_list(Goal0, TerminalsList),
	mongolog_expand(Goal0, Expanded, Mode).

%%
% Each conjunction with cut operator [X0,...,Xn,!|_]
% is rewritten as [limit(1,[X0,....,Xn])|_].
%
expand_cut([],[]) :- !.
expand_cut(Terms,Expanded) :-
	take_until_cut(Terms, Taken, Remaining),
	% no cut if Remaining=[]
	(	Remaining=[] -> Expanded=Terms
	% else the first element in Remaining must be a cut
	% that needs to be applied to goals in Taken
	;	(	Remaining=[!|WithoutCut],
			expand_cut(WithoutCut, Remaining_Expanded),
			Expanded=[limit(1,Taken)|Remaining_Expanded]
		)
	).


%% mongolog_compile(+Term, -Pipeline, +Context) is semidet.
%
% Translate a goal into an aggregation pipeline.
% Goal may be a compound term using the various predicates
% supported by mongolog.
% The goal must not but can be expanded before (see mongolog_expand/3).
% An error is thrown in case of compilation failure.
% One failure case is to refer to an unknown predicate
% (it is thus necessary to assert all referred predicates before
% compiling a new predicate).
% Such an error will also be thrown for recursive rules
% (i.e. when a predicate refers to itself).
%
% This predicate usually does not need to be called directly,
% but mongolog_assert/1 is called instead to make predicates
% accessible in other rules.
%
% @param Term A compound term, or a list of terms.
% @param Pipeline a term pipeline(Document,Variables)
% @param Context the query context
%
mongolog_compile(Terminals, pipeline(Doc, Vars), Context) :-
	catch(
		query_compile1(Terminals, Doc, Vars, Context),
		% catch error's, add context, and throw again
		error(Formal),
		throw(error(Formal,Terminals))
	).

%%
query_compile1(Terminals, Doc, Vars, Context) :-
	DocVars=[['g_assertions',_]],
	compile_terms(Terminals, Doc0, DocVars->Vars, _StepVars, Context),
	Doc=[['$set',['g_assertions',array([])]] | Doc0].

%%
compile_terms(Goal, Pipeline, Vars, StepVars, Context) :-
	\+ is_list(Goal), !,
	compile_terms([Goal], Pipeline, Vars, StepVars, Context).

compile_terms([], [], V0->V0, [], _) :- !.
compile_terms([X|Xs], Pipeline, V0->Vn, StepVars, Context) :-
	compile_term(X,  Pipeline_x,  V0->V1, StepVars0, Context),
	compile_terms(Xs, Pipeline_xs, V1->Vn, StepVars1, Context),
	append(Pipeline_x, Pipeline_xs, Pipeline),
	append(StepVars0, StepVars1, StepVars).

%% Compile a single command (Term) into an aggregate pipeline (Doc).
compile_term(Term, Doc, V0->V1, StepVars, Context) :-
	% try expansion (maybe the goal was not known during the expansion phase)
	option(mode(Mode), Context),
	mongolog_expand(Term, Expanded, Mode),
	compile_expanded_terms(Expanded, Doc, V0->V1, StepVars, Context).

%%
compile_expanded_terms(Goal, Doc, Vars, StepVars, Context) :-
	\+ is_list(Goal), !,
	compile_expanded_terms([Goal], Doc, Vars, StepVars, Context).

compile_expanded_terms([], [], V0->V0, [], _) :- !.
compile_expanded_terms([Expanded|Rest], Doc, V0->Vn, StepVars, Context) :-
	compile_expanded_term(Expanded, Doc0, V0->V1, StepVars0, Context),
	compile_expanded_terms(Rest, Doc1, V1->Vn, StepVars1, Context),
	append(Doc0, Doc1, Doc),
	append(StepVars0, StepVars1, StepVars).
	
compile_expanded_term(List, Doc, Vars, StepVars, Context) :-
	is_list(List),!,
	compile_expanded_terms(List, Doc, Vars, StepVars, Context).
	
compile_expanded_term(Expanded, Pipeline, V0->V1, StepVars_unique, Context) :-
	% create inner context
	merge_options([outer_vars(V0)], Context, InnerContext),
	% and finall compile expanded terms
	once(step_compile(Expanded, InnerContext, Doc, StepVars)),
	% merge StepVars with variables in previous steps (V0)
	list_to_set(StepVars, StepVars_unique),
	append(V0, StepVars_unique, V11),
	list_to_set(V11, V1),
	% create a field for each variable that was not referred to before
	findall([VarKey,[['type',string('var')], ['value',string(VarKey)]]],
		(	member([VarKey,_], StepVars_unique),
			\+ member([VarKey,_], V0)
		),
		VarDocs),
	(	VarDocs=[] -> Pipeline=Doc
	;	Pipeline=[['$set', VarDocs]|Doc]
	).

%%
step_compile(Step, Ctx, Doc, StepVars) :-
	step_compile(Step, Ctx, Doc),
	step_vars(Step, Ctx, StepVars).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% QUERY DOCUMENTS
%%%%%%%%%%%%%%%%%%%%%%%

%%
match_equals(X, Exp, Step) :-
	(	Step=['$set',   ['t_is_equal', ['$eq', array([X,Exp])]]]
	;	Step=['$match', ['t_is_equal', bool(true)]]
	;	Step=['$unset', string('t_is_equal')]
	).

%%
lookup_let_doc(InnerVars, LetDoc) :-
	findall([Key,string(Value)],
		(	member([Key,_], InnerVars),
			atom_concat('$',Key,Value)
		),
		LetDoc0),
	list_to_set(LetDoc0,LetDoc).

%%
lookup_set_vars(InnerVars, SetVars) :-
	% NOTE: let doc above ensures all vars can be accessed.
	%       this does also work if the let var was undefined.
	%       then the set below is basically a no-op.
	%       e.g. this runs through _without_ assigning "key" field:
	%
	%       db.one.aggregate([{'$lookup': {
	%			from: "one",
	%			as: "next",
	%			let: { "test": "$test"},
	%			pipeline: [{'$set': {"key": "$$test"}}]
	%		}}])
	%
	findall([Y,string(Y0)],
		(	member([Y,_], InnerVars),
			%member([Y,_], OuterVars),
			atom_concat('$$',Y,Y0)
		),
		SetVars0),
	list_to_set(SetVars0,SetVars).

%%
% find all records matching a query and store them
% in an array.
%
lookup_array(ArrayKey, Terminals,
		Prefix, Suffix,
		Context, StepVars,
		['$lookup', [
			['from', string(Coll)],
			['as', string(ArrayKey)],
			['let', LetDoc],
			['pipeline', array(Pipeline1)]
		]]) :-
	% get variables referred to in query
	option(outer_vars(OuterVars), Context),
	% join collection with single document
	mng_one_db(_DB, Coll),
	% generate inner pipeline
	compile_terms(Terminals, Pipeline,
		OuterVars->_InnerVars,
		StepVars, Context),
	(	option(additional_vars(AddVars), Context)
	->	append(AddVars, StepVars, StepVars0)
	;	StepVars0 = StepVars
	),
	%% FIXME: why needed?
	%  - it should be enough that triple yields these as step vars!
	%  - probably the problem is if triple is inside of disjunction!
	% FIXME: I suspect that context_var may not always yield same results for same context
	%
	once((
		bagof(Var,
			(	member(Var,StepVars0)
			;	context_var(Context, Var)
			),
			StepVars1)
	;	StepVars1=StepVars0
	)),
	% pass variables from outer scope to inner if they are referred to
	% in the inner scope.
	lookup_let_doc(StepVars1, LetDoc),
	% set all let variables so that they can be accessed
	% without aggregate operators in Pipeline
	lookup_set_vars(StepVars1, SetVars),
	% compose inner pipeline
	(	SetVars=[] -> Prefix0=Prefix
	;	Prefix0=[['$set', SetVars] | Prefix]
	),
	append(Prefix0,Pipeline,Pipeline0),
	% $set compile-time grounded vars for later unification.
	% this is needed because different branches cannot ground the same
	% variable to different values compile-time.
	findall([Key,TypedValue],
		(	member([Key,Val],StepVars1),
			ground(Val),
			mng_typed_value(Val,TypedValue)
		),
		GroundVars),
	(	GroundVars=[] -> Suffix0=Suffix
	;	Suffix0=[['$set', GroundVars] | Suffix]
	),
	append(Pipeline0,Suffix0,Pipeline1).

%%
lookup_next_unwind(Terminals,
		Prefix, Suffix,
		Ctx, Pipeline, StepVars) :-
	lookup_array('next', Terminals, Prefix, Suffix,
			Ctx, StepVars, Lookup),
	findall(Step,
		% generate steps
		(	Step=Lookup
		% unwind "next" field
		;	Step=['$unwind',string('$next')]
		% compute the intersection of scope
		% TODO: this should be optional, or better not handled here.
		%        - only if Terminas contains a triple we need to handle scope.
%		;	mng_scope_intersect('v_scope',
%				string('$next.v_scope.time.since'),
%				string('$next.v_scope.time.until'),
%				Ctx, Step)
		% set variables from "next" field
		;	set_next_vars(StepVars, Step)
		% remove "next" field again
		;	Step=['$unset',string('next')]
		),
		Pipeline),
	% the inner goal is not satisfiable if Pipeline==[]
	Pipeline \== [].

%%
% Move ground variables in "next" document to the
% document root.
% However, not all variables referred to in the goal may
% have a grounding, so we need to make a conditional $set here.
%
set_next_vars(InnerVars, ['$set', [Key,
		['$cond',array([
			['$not', array([string(NewVal)])], % if the field does not exist in 'next'
			string(OldVal),                    % set the field to its current value
			string(NewVal)                     % else overwrite with value in next
		])]]]) :-
	findall(Key0, member([Key0,_], InnerVars), Keys0),
	list_to_set(Keys0, Keys),
	member(Key, Keys),
	atom_concat('$',Key,OldVal),
	atom_concat('$next.',Key,NewVal).

%%
add_assertions(Docs, Coll,
	['$set', ['g_assertions',['$concatArrays', array([
		string('$g_assertions'),
		array([[
			['collection', string(Coll)],
			['documents', Docs]
		]])
	])]]]).

add_assertion(Doc, Coll, Step) :-
	add_assertions(array([Doc]), Coll, Step).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% VARIABLES in queries
%%%%%%%%%%%%%%%%%%%%%%%

%%
add_assertion_var(StepVars, Ctx, [['g_assertions',_]|StepVars]) :-
	(	option(mode(tell), Ctx)
	;	option(retract, Ctx)
	),!.
add_assertion_var(StepVars, _, StepVars).

% read all variables referred to in Step into list StepVars
step_vars(Step, Ctx, StepVars) :-
	(	bagof(Vs, goal_var(Step, Ctx, Vs), StepVars)
	;	StepVars=[]
	),!.

%%
goal_var(Var, Ctx, [Key,Var]) :-
	var(Var),!,
	var_key(Var, Ctx, Key).

goal_var(List, Ctx, Var) :-
	is_list(List),!,
	member(X,List),
	goal_var(X, Ctx, Var).

goal_var(Dict, Ctx, Var) :-
	is_dict(Dict),!,
	get_dict(Key, Dict, Value),
	(	goal_var(Key,Ctx,Var)
	;	goal_var(Value,Ctx,Var)
	).

goal_var(Compound, Ctx, Var) :-
	compound(Compound),!,
	Compound =.. [_Functor|Args],
	member(Arg,Args),
	goal_var(Arg, Ctx, Var).

%%
context_var(Ctx, [Key,ReferredVar]) :-
	option(scope(Scope), Ctx),
	% NOTE: vars are resolved to keys in scope already!
	%       e.g. `Since == =<(string($v_235472))`
	time_scope(Since, Until, Scope),
	member(X, [Since, Until]),
	mng_strip(X, _, string, Stripped),
	atom(Stripped),
	atom_concat('$', Key, Stripped),
	once((
		(	option(outer_vars(Vars), Ctx)
		;	option(disj_vars(Vars), Ctx)
		),
		member([Key,ReferredVar],Vars)
	)).

%%
% Conditional $set command for ungrounded vars.
%
set_if_var(X, Exp, Ctx,
		['$set', [Key, ['$cond', array([
			% if X is a variable
			['$eq', array([string(TypeVal), string('var')])],
			Exp,          % evaluate the expression and set new value
			string(XVal)  % else value remains the same
		])]]]) :-
	var_key(X, Ctx, Key),
	atom_concat('$',Key,XVal),
	atom_concat(XVal,'.type',TypeVal).

%%
get_var(Term, Ctx, [Key,Var]) :-
	term_variables(Term,Vars),
	member(Var,Vars),
	var_key(Var, Ctx, Key).

%%
% Map a Prolog variable to the key that used to
% refer to this variable in mongo queries.
%
var_key(Var, Ctx, Key) :-
	var(Var),
	(	option(outer_vars(Vars), Ctx)
	;	option(disj_vars(Vars), Ctx)
	),
	member([Key,ReferredVar],Vars),
	ReferredVar == Var,
	!.
var_key(Var, _Ctx, Key) :-
	var(Var),
	term_to_atom(Var,Atom),
	atom_concat('v',Atom,Key).

%%
% yield either the key of a variable in mongo,
% or a typed term for some constant value provided
% in the query.
%
var_key_or_val(In, Ctx, Out) :-
	mng_strip_operator(In, '=', In0),
	var_key_or_val0(In0, Ctx, Out).
	
var_key_or_val0(In, Ctx, string(Key)) :-
	mng_strip_type(In, _, In0),
	var_key(In0, Ctx, Out),
	atom_concat('$',Out,Key),
	!.

var_key_or_val0(In, _Ctx, Out) :-
	atomic(In),!,
	once(get_constant(In,Out)).

var_key_or_val0(In, Ctx, array(L)) :-
	is_list(In),!,
	findall(X,
		(	member(Y,In),
			var_key_or_val0(Y, Ctx, X)
		),
		L).

var_key_or_val0(:(NS,Atom), _, _) :-
	throw(unexpanded_namespace(NS,Atom)).

var_key_or_val0(TypedValue, _Ctx, TypedValue) :-
	compound(TypedValue),
	TypedValue =.. [Type|_],
	mng_client:type_mapping(Type,_),
	!.

var_key_or_val0(Term, Ctx, [
		['type', string('compound')],
		['value', [
			['functor', string(Functor)],
			['args', array(Vals)]
		]]
	]) :-
	mng_strip_type(Term, term, Stripped),
	compound(Stripped),
	Stripped =.. [Functor|Args],
	findall(X,
		(	member(Y,Args),
			var_key_or_val0(Y, Ctx, X)
		),
		Vals).

var_key_or_val1(In, Ctx, Out) :-
	var_key_or_val(In, Ctx, X),
	(	(X=string(Str), atom(Str), atom_concat('$',_,Str))
	->	(X=string(Str), atom_concat('$',Str,Y), Out=string(Y))
	;	Out=X
	).

%% in case of atomic in query
get_constant(Value, double(Value)) :- number(Value).
get_constant(true,  bool(true)).
get_constant(false, bool(false)).
get_constant(Value, string(Value)) :- atom(Value).
get_constant(Value, string(Value)) :- string(Value).

%%
% True iff Arg has been referred to in the query before.
% That is, Arg has been added to the "outer variables"
% of the compile context.
%
is_referenced(Arg, Ctx) :-
	option(outer_vars(OuterVars),Ctx),
	var_key(Arg, Ctx, Key),
	memberchk([Key,_], OuterVars).

%%
all_ground(Args, Ctx) :-
	forall(
		member(Arg,Args),
		(	is_instantiated(Arg, Ctx) -> true
		;	throw(error(instantiation_error))
		)
	).

is_instantiated(Arg, Ctx) :-
	mng_strip_variable(Arg, Arg0),
	term_variables(Arg0, Vars),
	forall(
		member(Var, Vars),
		is_referenced(Var, Ctx)
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
% split list at cut operator.
%
take_until_cut([],[],[]) :- !.
take_until_cut(['!'|Xs],[],['!'|Xs]) :- !.
take_until_cut([X|Xs],[X|Ys],Remaining) :-
	take_until_cut(Xs,Ys,Remaining).

%%
% do not use is_list/1 here, it cannot handle list that have not
% been completely resolved as in `[a|_]`.
% Here we check just the head of the list.
%
has_list_head([]) :- !.
has_list_head([_|_]).

