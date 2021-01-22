:- module(query_compiler,
	[ query_assert(t),
	  query_ask(t,+,-,+),
	  query_tell(t,+,+),
	  query_expand/3,
	  query_compile/3
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

% TODO: cut operator
%     -- 1. within a clause `cut` wraps terms before
%             in a lookup with $limit
%     -- 2. disjunctions keep track of cut in disjuncts
%             and does not proceed if cut+matching docs in previous step

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1
	    ]).

%% Stores list of terminal terms for each clause. 
:- dynamic query/4.
%% set of registered query commands.
:- dynamic step_command/1.
%% optionally implemented by query commands.
:- multifile step_expand/3.
%% implemented by query commands to compile query documents
:- multifile step_compile/3.
%% implemented by query commands to provide variables exposed to the outside
:- multifile step_var/2.

:- rdf_meta(step_compile(t,t,t)).

%% add_command(+Command) is det.
%
% register a command that can be used in KnowRob
% language expressions and which is implemented
% in a mongo query.
% NOTE: to implement a command several multifile predicates in
% mng_query and mng_compiler must be implemented by a command. 
%
% @Command a command term.
%
add_command(Command) :-
	assertz(step_command(Command)).


%% mng_ask(+Statement, +Scope, +Options) is nondet.
%
% Run a mongo query to find out if Statement holds
% within Scope.
%
% @Statement a statement written in KnowRob language
% @Scope the scope of the statement
% @Options query options
%
query_ask(Statement, QScope, FScope, Options) :-
	query_(Statement,
		[scope(QScope)|Options],
		FScope, ask).

%% mng_tell(+Statement, +Scope, +Options) is semidet.
%
% Run a mongo query to assert that a Statement holds
% within Scope.
%
% @Statement a statement written in KnowRob language
% @Scope the scope of the statement
% @Options query options
%
query_tell(Statement, FScope, Options) :-
	query_(Statement,
		[scope(FScope)|Options],
		_, tell).

%%
query_(Goal, Context, FScope, Mode) :-
	% expand goals into terminal symbols
	query_expand(Goal, Expanded, Mode),
	% get the pipeline document
	query_compile(Expanded, pipeline(Doc,Vars), [Mode|Context]),
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
	unify_0(Result, Vars).

unify_0(_, []) :- !.
unify_0(Doc, [X|Xs]) :-
	once(unify_1(Doc, X)),
	unify_0(Doc, Xs).

unify_1(_, ['_id', _]).
unify_1(Doc, [VarKey, Term]) :-
	nonvar(Term),
	Term=list(Var,Pattern),
	!,
	mng_get_dict(VarKey, Doc, ArrayValue),
	unify_list(ArrayValue, Pattern, Var).

unify_1(Doc, [VarKey, Var]) :-
	mng_get_dict(VarKey, Doc, TypedValue),
	mng_strip_type(TypedValue, _, Value),
	(	Value='null' ;
		Var=Value
	).

%%
unify_list(array([]),_,[]) :- !.
unify_list(array([X|Xs]),Pattern,[Y|Ys]) :-
	dict_pairs(Dict, _, X),
	unify_list_1(Dict, Pattern, Y),
	unify_list(array(Xs), Pattern, Ys).

%%
unify_list_1(_, Ground, Ground) :-
	ground(Ground),!.

unify_list_1(Doc, Var, Elem) :-
	var(Var),!,
	var_key(Var, Key),
	unify_1(Doc, [Key, Elem]).

unify_list_1(Doc, List, Elem) :-
	is_list(List),!,
	findall(Y,
		(	member(X,List),
			unify_list_1(Doc,X,Y)
		),
		Elem).

unify_list_1(Doc, Term, Elem) :-
	compound(Term),!,
	Term =.. List0,
	unify_list_1(Doc, List0, List1),
	Elem =.. List1.

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
	(	query_expand(Body, Expanded, Context)
	->	true
	;	log_error_and_fail(lang(assertion_failed(Body), Functor))
	),
	log_debug(lang(expanded(Functor, Args, Expanded, Context))),
	%% store expanded query
	assertz(query(Functor, Args, Expanded, Context)).

%% query_expand(+Goal, -Expanded, +Context) is det.
%
% Translates a KnowRob langauge term into a sequence
% of commands that can be executed by mongo DB.
%
% @Goal a KnowRob language term
% @Expanded sequence of commands
% @Context 'ask' or 'tell'
%
query_expand(Goal, Expanded, Context) :-
	comma_list(Goal, Terms),
	catch(
		expand_term_0(Terms, Expanded, Context),
		Exc,
		log_error_and_fail(lang(Exc, Goal))
	).

%%
expand_term_0([], [], _Context) :- !.
expand_term_0([X|Xs], Expanded, Context) :-
	expand_term_1(X, X_expanded, Context),
	expand_term_0(Xs, Xs_expanded, Context),
	append(X_expanded, Xs_expanded, Expanded).

%% try expanding disjuntion, else call expand_term_2
expand_term_1(Goal, Expanded, Context) :-
	semicolon_list(Goal, Terms),
	(	Terms=[_]
	->	expand_term_2(Goal, Expanded, Context)
	;	expand_term_2(facet(Terms), Expanded, Context)
	).

%% strip all modifiers from the goal and call expand_term_3
expand_term_2(Goal, Expanded, Context) :-
	strip_all_modifier(Goal, Stripped, Modifier),
	expand_term_3(Stripped, Modifier, Expanded, Context).

%% handle goals wrapped in call.
expand_term_3(call(Goal), _Modifier, Expanded, Context) :-
	!,
	mng_expand(Goal, Expanded, Context).

%% handle goals wrapped in ask. this is especially used for conditional tel clauses.
expand_term_3(ask(Goal), _Modifier, Expanded, Context) :-
	(	select_option(tell, Context, Context0)
	->	Context1=[ask|Context0]
	;	Context1=Context
	),
	!,
	query_expand(Goal, Expanded, Context1).

%% finally expand rules that were asserted before
expand_term_3(Goal, Modifier, [step(Expanded,Modifier)], Context) :-
	Goal =.. [Functor|_Args],
	step_command(Functor),
	!,
	% TODO: verify that Modifier are supported by command
	(	step_expand(Goal, Expanded, Context)
	->	true
	;	Expanded = Goal
	).

expand_term_3(Goal, Modifier0, Expanded, Context) :-
	% find all asserted rules matching the functor and args
	Goal =.. [Functor|Args],
	% NOTE: do not use findall here because findall would not preserve
	%       variables in Terminals
	% TODO: do we need to create a copy of Terminals here, or not needed?
	(	bagof(Terminals,
			query(Functor, Args, Terminals, Context),
			TerminalsList)
	->	true
	% handle the case that a predicate is referred to that wasn't
	% asserted before
	;	throw(expansion_failed(Goal))
	),
	% need to wrap in facet/1 to indicate that there are multiple clauses.
	% the case of multiple clauses is handled using the $facet command.
	% TODO: how should modifier be handled? I think it would
	%        not be appropiate to just apply them to every command in TerminalsList
	%    - ignore/once/limit coud be handled by wrapping everything in a $lookup
	%    - transitive/reflexive is more difficult
	%
	(	TerminalsList=[List]
	->	Expanded = List
	;	Expanded = [step(facet(TerminalsList),Modifier0)]
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
		compile_0(Terminals, Doc, []->Vars, Context),
		Exc,
		log_error_and_fail(lang(Exc, Terminals))
	).

%%
compile_0(Terminals, Doc, Vars, Context) :-
	compile_1(Terminals, Doc0, Vars, Context),
	(	memberchk(ask, Context)
	->	compile_ask_(Doc0, Doc)
	;	compile_tell_(Doc0, Doc)
	).

%%
compile_1([], [], V0->V0, _) :- !.
compile_1([X|Xs], Pipeline, V0->Vn, Context) :-
	compile_2(X,  Pipeline_x,  V0->V1, Context),
	compile_1(Xs, Pipeline_xs, V1->Vn, Context),
	% TODO: avoid append
	append(Pipeline_x, Pipeline_xs, Pipeline).

%% Compile a single command (Term) into an aggregate pipeline (Doc).
compile_2(step(Term,Modifier), Doc, V0->V1, Context) :-
	% read all variables referred to in Step into list StepVars
	(	bagof(Vs, step_var(Term, Vs), StepVars)
	->	true
	;	StepVars=[]
	),
	% merge StepVars with variables in previous steps (V0)
	append(V0, StepVars, Vars_new),
	list_to_set(Vars_new, V1),
	% add modifier to context
	append(Modifier, Context, InnerContext),
	% compile JSON document for this step
	once(step_compile(Term, [
			step_vars(StepVars),
			outer_vars(V0) |
			InnerContext
	], Doc)).

%%
compile_ask_(Doc, Doc) :-
	!.

%%
compile_tell_(Doc0, Doc1) :-
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
%%%%%%%%% MODIFIER
%%%%%%%%%%%%%%%%%%%%%%%

%% modifiers are stripped from terms and stored separately
strip_all_modifier(In, Out, [Mod0|Mods]) :-
	strip_modifier(In,In0,Mod0),
	!,
	strip_all_modifier(In0,Out,Mods).
strip_all_modifier(In, In, []).

%%
strip_modifier(ignore(X),ignore,X).
strip_modifier(once(X),limit(1),X).
strip_modifier(limit(X,N),limit(N),X).
strip_modifier(transitive(X),transitive,X).
strip_modifier(reflexive(X),reflexive,X).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% VARIABLES in queries
%%%%%%%%%%%%%%%%%%%%%%%

%%
% Map a Prolog variable to the key that used to
% refer to this variable in mongo queries.
%
var_key(Var,Key) :-
	var(Var),
	term_to_atom(Var,Atom),
	atom_concat('v',Atom,Key).

%%
% yield either the key of a variable in mongo,
% or a typed term for some constant value provided
% in the query.
%
var_key_or_val(In, Out) :-
	mng_strip_operator(In, Operator, In0),
	var_key_or_val0(In0, Out0),
	mng_strip_operator(Out, Operator, Out0).
	
var_key_or_val0(In, string(Key)) :-
	mng_strip_type(In, _, In0),
	var_key(In0, Out),
	atom_concat('$',Out,Key),
	!.

var_key_or_val0(In, Out) :-
	atomic(In),!,
	once(get_constant_(In,Out)).

var_key_or_val0(In, array(L)) :-
	is_list(In),!,
	findall(X,
		(	member(Y,In),
			var_key_or_val0(Y,X)
		),
		L).

var_key_or_val0(In,In) :-
	compound(In).

%% in case of atomic in query
get_constant_(Value, double(Value)) :- number(Value).
get_constant_(true,  bool(true)).
get_constant_(false, bool(false)).
get_constant_(Value, string(Value)) :- atom(Value).
get_constant_(Value, string(Value)) :- string(Value).

