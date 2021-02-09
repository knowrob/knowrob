:- module(query_compiler,
	[ query_assert(t),
	  query_ask(t,+,-,+),
	  query_tell(t,+,+),
	  query_expand(t,-,+),
	  query_compile(t,-,+)
	]).
/** <module> Compilation of KnowRob rules into DB queries.

KnowRob language terms are defined in rules which are translated
into aggregate pipelines that can be processed by mongo DB.

@author Daniel Beßler
@license BSD
*/

% TODO: recursion in rules
%		- transitive(triple) performs recursive $graphLookup
%		- $graphLookup has limits, what exactly are these?
%		- how to handle transitive chains i.e. transitive(p1 o ... o pn)?
%		- how to handle more complex structures?

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1 ]).
:- use_module(library('db/mongo/client')).

%% Stores list of terminal terms for each clause. 
:- dynamic query/4.
%% set of registered query commands.
:- dynamic step_command/1.
%% optionally implemented by query commands.
:- multifile step_expand/3.
%% implemented by query commands to compile query documents
:- multifile step_compile/3.
%% implemented by query commands to provide variables exposed to the outside
:- multifile step_vars/3.

:- rdf_meta(step_compile(t,t,t)).

%% add_command(+Command) is det.
%
% register a command that can be used in KnowRob
% language expressions and which is implemented
% in a mongo query.
% NOTE: to implement a command several multifile predicates in
% query_compiler must be implemented by a command. 
%
% @Command a command term.
%
add_command(Command) :-
	assertz(step_command(Command)).


%% query_ask(+Statement, +Scope, +Options) is nondet.
%
% Run a mongo query to find out if Statement holds
% within Scope.
%
% @Statement a statement written in KnowRob language
% @Scope the scope of the statement
% @Options query options
%
query_ask(Statement, QScope, FScope, Options) :-
	query_(Statement, [scope(QScope)|Options], FScope, ask).

%% query_tell(+Statement, +Scope, +Options) is semidet.
%
% Run a mongo query to assert that a Statement holds
% within Scope.
%
% @Statement a statement written in KnowRob language
% @Scope the scope of the statement
% @Options query options
%
query_tell(Statement, FScope, Options) :-
	query_(Statement, [scope(FScope)|Options], _, tell).

%%
query_(Goal, Context, FScope, Mode) :-
	% expand goals into terminal symbols
	query_expand(Goal, Expanded, Mode),
	%% FIXME BUG in some cases SWIPL replaces variables in the compile step
	%            below. But it does not happen when running with gtrace,
	%            and also when adding writeln calls. very strange...
	%            for some reason it does not appear to happen when doing something
	%            with the vars here at the toplevel?!?
	touch_term_vars(Expanded),
	% get the pipeline document
	query_compile(Expanded, pipeline(Doc,Vars), [mode(Mode)|Context]),
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
query_2(ask, Cursor, Vars, FScope) :-
	!,
	mng_cursor_materialize(Cursor, Result),
	unify_(Result, Vars, FScope).

query_2(tell, Cursor, _Vars, _FScope) :-
	!,
	% first, run the query.
	% NOTE: tell cannot materialize the cursor
	% because there are no output documents (due to $merge command)
	mng_cursor_run(Cursor),
	% second, remove all documents that have been flagged by the aggregate pipeline.
	% this is needed because it seems not possible to merge+delete
	% in one pipeline, so the first pass just tags the documents
	% that need to be removed.
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, ['delete', bool(true)]).

%%
% read accumulated fact scope and
% unify variables.
%
unify_(Result, Vars, FScope) :-
	mng_get_dict('v_scope', Result, FScope),
	%unify_vars(Result, Vars, Vars),
	unify_0(Result, Vars, Vars).

unify_0(_, _, []) :- !.
unify_0(Doc, Vars, [X|Xs]) :-
	% it is ignored here if some variables referred
	% to are not properly grounded.
	% this can happen e.g. in expressions such as (A=a;B=b)
	% where the first solution has grounded A but not B.
	ignore(unify_1(Doc, Vars, X)),
	unify_0(Doc, Vars, Xs).

unify_1(_, _, ['_id', _]).

unify_1(_, _, [_, Term]) :-
	% variable was unified in pragma command
	ground(Term), !.

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

%%

touch_term_vars(Expanded) :-
	term_variables(Expanded, ExpandedVars),
	forall(
		member(ExpandedVar,ExpandedVars),
		put_attr(ExpandedVar, test, 1)
	).

%% query_assert(+Rule) is semidet.
%
% Asserts a rule executable in a mongo query.
% The rule is internally translated into a form
% that only contains asequence of commands that can be executed
% by mongo.
% This sequence of commands can later be executed
% as a mongo aggregate query.
% `Rule` can be either a term `Head ?> Body` or `Head +> Body`.
%
% Asserting rule during term expansion has the benefit
% of being slightly faster compared to doing the translation
% at runtime (which is also perfectly possible).
%
% @Rule the rule to assert.
%
query_assert((?>(Head,Body))) :-
	query_assert1(Head, Body, ask).

query_assert((+>(Head,Body))) :-
	query_assert1(Head, Body, tell).

query_assert1(Head, Body, Context) :-
	%% get the functor of the predicate
	Head =.. [Functor|Args],
	%% expand goals into terminal symbols
	(	query_expand(Body, Expanded, Context) -> true
	;	log_error_and_fail(lang(assertion_failed(Body), Functor))
	),
	%% handle instantiated arguments
	(	expand_arguments(Args, ExpandedArgs, ArgsGoal)
	->	(Expanded0=[ArgsGoal|Expanded], Args0=ExpandedArgs)
	;	(Expanded0=Expanded, Args0=Args)
	),
	%%
	log_debug(lang(expanded(Functor, Args0, Expanded0, Context))),
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

%% query_expand(+Goal, -Expanded, +Mode) is det.
%
% Translates a KnowRob langauge term into a sequence
% of commands that can be executed by mongo DB.
%
% @Goal a KnowRob language term
% @Expanded sequence of commands
% @Mode 'ask' or 'tell'
%
query_expand(Goal, Goal, _) :-
	% goals maybe not known during expansion, i.e. in case of
	% higher-level predicates receiving a goal as an argument.
	% these var goals need to be expanded compile-time
	% (call-time is not possible)
	%
	var(Goal), !.

query_expand(Goal, Expanded, Mode) :-
	% NOTE: do not use is_list/1 here, it cannot handle list that have not
	%       been completely resolved as in `[a|_]`.
	%       Here we chack just the head of the list.
	\+ has_list_head(Goal), !,
	comma_list(Goal, Terms),
	query_expand(Terms, Expanded, Mode).

query_expand(Terms, Expanded, Mode) :-
	has_list_head(Terms), !,
	catch(
		expand_term_0(Terms, Expanded0, Mode),
		Exc,
		log_error_and_fail(lang(Exc, Terms))
	),
	% Handle cut after term expansion.
	% It is important that this is done _after_ expansion because
	% the cut within the call would yield an infinite recursion
	% otherwhise.
	% TODO: it is not so nice doing it here. would be better if it could be
	%       done in control.pl where cut operator is implemented but current
	%       interfaces don't allow to do the following operation in control.pl.
	%		(without special handling of ',' it could be done, I think)
	expand_cut(Expanded0, Expanded1),
	%%
	(	Expanded1=[One] -> Expanded=One
	;	Expanded=Expanded1
	).

%%
expand_term_0([], [], _Mode) :- !.
expand_term_0([X|Xs], [X_expanded|Xs_expanded], Mode) :-
	once(expand_term_1(X, X_expanded, Mode)),
	% could be that expand-time the list is not fully resolved
	(	var(Xs) -> Xs_expanded=Xs
	;	expand_term_0(Xs, Xs_expanded, Mode)
	).

%% handle goals wrapped in ask. this can be used for conditional tell clauses.
expand_term_1(ask(Goal), Expanded, _Mode) :-
	query_expand(Goal, Expanded, ask).

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
	(	bagof(Terminals,
			query(Functor, Args, Terminals, Mode),
			TerminalsList)
	->	true
	% handle the case that a predicate is referred to that wasn't
	% asserted before
	;	throw(expansion_failed(Goal,Mode))
	),
	% wrap different clauses into ';'
	semicolon_list(Expanded, TerminalsList).

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


%% query_compile(+Terminals, -Pipeline, +Context) is semidet.
%
% Compile an aggregate pipeline given a list of terminal symbols
% and the context in which they shall hold.
%
% @Terminals list of terminal symbols
% @Pipeline a term pipeline(Doc,Vars)
% @Context the query context
%
query_compile(Terminals, pipeline(Doc, Vars), Context) :-
	catch(
		query_compile1(Terminals, Doc, []->Vars, Context),
		Exc,
		log_error_and_fail(lang(Exc, Terminals))
	).

%%
query_compile1(Terminals, Doc, Vars, Context) :-
	compile_terms(Terminals, Doc0, Vars, Context),
	(	memberchk(mode(ask), Context) -> Doc=Doc0
	;	compile_tell(Doc0, Doc)
	).

%%
compile_terms(Goal, Pipeline, Vars, Context) :-
	\+ is_list(Goal), !,
	compile_terms([Goal], Pipeline, Vars, Context).

compile_terms([], [], V0->V0, _) :- !.
compile_terms([X|Xs], Pipeline, V0->Vn, Context) :-
	compile_term(X,  Pipeline_x,  V0->V1, Context),
	compile_terms(Xs, Pipeline_xs, V1->Vn, Context),
	append(Pipeline_x, Pipeline_xs, Pipeline).

%% Compile a single command (Term) into an aggregate pipeline (Doc).
compile_term(Term, Doc, V0->V1, Context) :-
	% try expansion (maybe the goal was not known during the expansion phase)
	query_expand(Term, Expanded, ask),
	compile_expanded_terms(Expanded, Doc, V0->V1, Context).

%%
compile_expanded_terms(Goal, Doc, Vars, Context) :-
	\+ is_list(Goal), !,
	compile_expanded_terms([Goal], Doc, Vars, Context).

compile_expanded_terms([], [], V0->V0, _) :- !.
compile_expanded_terms([Expanded|Rest], Doc, V0->Vn, Context) :-
	compile_expanded_term(Expanded, Doc0, V0->V1, Context),
	compile_expanded_terms(Rest, Doc1, V1->Vn, Context),
	append(Doc0, Doc1, Doc).
	
compile_expanded_term(Expanded, Pipeline, V0->V1, Context) :-
	% read all variables referred to in Step into list StepVars
	(	bagof(Vs, goal_var(Expanded, Context, Vs), StepVars) -> true
	;	StepVars=[]
	),
	list_to_set(StepVars, StepVars_unique),
	% create a field for each variable that was not referred to before
	findall([VarKey,[['type',string('var')], ['value',string(VarKey)]]],
		(	member([VarKey,_], StepVars_unique),
			\+ member([VarKey,_], V0)
		),
		VarDocs),
	(	VarDocs=[] -> Pipeline=Doc
	;	Pipeline=[['$set', VarDocs]|Doc]
	),
	% merge StepVars with variables in previous steps (V0)
	append(V0, StepVars_unique, V11),
	list_to_set(V11, V1),
	% create inner context
	merge_options(
		[step_vars(StepVars_unique),outer_vars(V0)],
		Context, InnerContext),
	% and finall compile expanded terms
	once(step_compile(Expanded, InnerContext, Doc)).

%%
compile_tell(Doc0, Doc1) :-
	% tell merges into triples DB
	mng_get_db(_DB, Coll, 'triples'),
	% append some steps to Doc0
	findall(Step,
		% create an empty array in 'triples' field
		(	Step=['$set',['triples',array([])]]
		% draw steps from language expression
		;	member(Step,Doc0)
		% the "triples" field holds an array of documents to be merged
		;	Step=['$unwind',string('$triples')]
		% make unwinded triple root of the document
		;	Step=['$replaceRoot',['newRoot',string('$triples')]]
		% merge document into triples collection
		% NOTE: $merge must be last step
		;	Step=['$merge',string(Coll)]
		),
		Doc1
	).

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
lookup_let_doc(InnerVars, OuterVars, LetDoc) :-
	findall([Key,string(Value)],
		(	member([Key,_], InnerVars),
			member([Key,_], OuterVars),
			atom_concat('$',Key,Value)
		),
		LetDoc).

%%
lookup_set_vars(InnerVars, OuterVars, SetVars) :-
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
			member([Y,_], OuterVars),
			atom_concat('$$',Y,Y0)
		),
		SetVars).

%%
% find all records matching a query and store them
% in an array.
%
lookup_array(ArrayKey, Terminals,
		Prefix, Suffix,
		Context, InnerVars,
		['$lookup', [
			['from', string(Coll)],
			['as', string(ArrayKey)],
			['let', LetDoc],
			['pipeline', array(Pipeline1)]
		]]) :-
	% get variables referred to in query
	option(outer_vars(OuterVars), Context),
	option(step_vars(StepVars), Context),
	% join collection with single document
	mng_one_db(_DB, Coll),
	% generate inner pipeline
	compile_terms(Terminals,
		Pipeline,
		OuterVars->InnerVars,
		Context),
	% pass variables from outer scope to inner if they are referred to
	% in the inner scope.
	lookup_let_doc(InnerVars, OuterVars, LetDoc),
	% set all let variables so that they can be accessed
	% without aggregate operators in Pipeline
	lookup_set_vars(InnerVars, OuterVars, SetVars),
	% compose inner pipeline
	(	SetVars=[] -> Prefix0=Prefix
	;	Prefix0=[['$set', SetVars] | Prefix]
	),
	append(Prefix0,Pipeline,Pipeline0),
	% $set compile-time grounded vars for later unification.
	% this is needed because different branches cannot ground the same
	% variable to different values compile-time.
	findall([Key,TypedValue],
		(	member([Key,Val],StepVars),
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
		Context, Step) :-
	lookup_array('next', Terminals, Prefix, Suffix,
			Context, InnerVars, Lookup),
	% generate steps
	(	Step=Lookup
	% unwind "next" field
	;	Step=['$unwind',string('$next')]
	% compute the intersection of scope
	;	mng_scope_intersect('v_scope',
			string('$next.v_scope.time.since'),
			string('$next.v_scope.time.until'),
			Context, Step)
	% set variables from "next" field
	;	set_next_vars(InnerVars, Step)
	% remove "next" field again
	;	Step=['$unset',string('next')]
	).

%%
% Move ground variables in "next" document to the
% document root.
% However, not all variables referred to in the goal may
% have a grounding, so we need to make a conditional $set here.
%
set_next_vars(InnerVars, ['$set', [Key,
		['$cond',array([
			% FIXME: should be is_var test here or?
			['$not', array([string(Val)])], % if the field does not exist
			[['type',string('var')], ['value',string('_')]],
			string(Val)                     % else write Val into field at Key
		])]]]) :-
	member([Key,_], InnerVars),
	atom_concat('$next.',Key,Val).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% VARIABLES in queries
%%%%%%%%%%%%%%%%%%%%%%%

%%
goal_var(Var, Ctx, [Key,Var]) :-
	var(Var),!,
	var_key(Var, Ctx, Key).

goal_var(Goal, Ctx, Var) :-
	step_vars(Goal, Ctx, Vars),!,
	member(Var,Vars).

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
% Conditional $set command for ungrounded vars.
%
set_if_var(X, Exp, Ctx,
		['$set', [Key, ['$cond', array([
			% if X is a variable
			['$eq', array([string(TypeVal), string('var')])],
			% evaluate the expression and set new value
			Exp,
			% else value remains the same
			string(XVal)
		])]]]) :-
	query_compiler:var_key(X, Ctx, Key),
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
	(	option(step_vars(Vars), Ctx)
	;	option(outer_vars(Vars), Ctx)
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

