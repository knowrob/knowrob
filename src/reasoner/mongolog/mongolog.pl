:- module(mongolog,
	[ mongolog_call(t),
	  mongolog_call(t,+),
	  mongolog_assert(t),
	  mongolog_assert(t,t),			% +Statement, +Scope
	  mongolog_project(t),
	  mongolog_retract(t),        % +Statement
	  mongolog_retract(t,t),      % +Statement, +Scope
	  mongolog_retract(t,t,t),    % +Statement, +Scope, +Options
	  mongolog_add_rule(t,t),
	  mongolog_drop_rule(t),
	  mongolog_expand(t,-),
	  mongolog_current_predicate/1,
	  setup_collection/2
	]).
/** <module> Compiling goals into aggregation pipelines.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('logging')).
:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1,
	      rdf_global_term/2 ]).
:- use_module('client').
:- use_module(library('scope')).
:- use_module(library('reasoner'),
        [ reasoner_setting/4,
          reasoner_setting/2,
          current_reasoner_module/1,
          reasoner_define_relation/2 ]).

%% set of registered query commands.
:- dynamic step_command/2.
% optionally implemented by query commands.
:- multifile step_expand/2.
%% implemented by query commands to compile query documents
:- multifile step_compile/3, step_compile/4.
%%
:- dynamic mongolog_rule/4.
%% maps source file URI to name of module in which it was loaded
:- dynamic mongolog_source_file/2.

%% operators for tell/ask rules
:- op(1100, xfx, user:(?>)).
:- op(1100, xfx, user:(+>)).
:- op(1100, xfx, user:(?+>)).

:- rdf_meta(step_compile(t,t,t)).
:- rdf_meta(step_compile(t,t,t,-)).

%% mongolog_current_predicate(+PredicateIndicator) is semidet.
%
% True if Predicate is a currently defined predicated.
%
mongolog_current_predicate(PredicateIndicator) :-
    mongolog_current_predicate(PredicateIndicator, _).

%%
mongolog_current_predicate(PredicateIndicator, PredicateType) :-
    ground(PredicateIndicator),
	strip_module_(PredicateIndicator, Module, Stripped),
	(   Stripped=(Functor/Arity) -> true
	;   atom(PredicateIndicator) -> Functor=PredicateIndicator
	),
	(   ground(Module) -> CommandModule = Module
	;   current_reasoner_module(CommandModule)
	),
	mongolog_current_predicate1(CommandModule, Functor, Arity, PredicateType).

%%
mongolog_current_predicate1(_Module, Functor, _Arity, built_in) :-
	% TODO: take arity into account. Same below
    step_command(user, Functor), !.

mongolog_current_predicate1(Module, Functor, _Arity, idb_relation) :-
    step_command(Module, Functor), !.

mongolog_current_predicate1(_Module, Functor, Arity, built_in) :-
    mongolog_rule(user, Functor, Args, _),
    length(Args, Arity),!.

mongolog_current_predicate1(Module, Functor, Arity, idb_relation) :-
    mongolog_rule(Module, Functor, Args, _),
    length(Args, Arity),!.

%% add_command(+Command) is det.
%
% Same as add_command/2 but uses "user" as command module.
%
% @param Command a command term.
%
add_command(Command) :- add_command(Command, user).

%% add_command(+Command, +CommandModule) is det.
%
% Defines a built-in command in mongolog.
%
% @param Command a command term.
% @param CommandModule the module in which the command is registered.
%
add_command(Command, _CommandModule) :-
    step_command(user,Command), !.
add_command(Command, CommandModule) :-
    step_command(CommandModule,Command), !.
add_command(Command, CommandModule) :-
    assertz(step_command(CommandModule,Command)).

%%
is_step_command(CommandModule, (/(Functor,_Arity))) :-
	!, step_command1(CommandModule,Functor).

is_step_command(CommandModule, Goal) :-
	compound(Goal),!,
	Goal =.. [Functor|_Args],
	step_command1(CommandModule,Functor).

is_step_command(CommandModule, Functor) :-
	atom(Functor),!,
	step_command1(CommandModule,Functor).

%%
step_command1(ReasonerModule, Functor) :-
    step_command(RealReasonerModule, Functor),
    once((RealReasonerModule==user ; RealReasonerModule==ReasonerModule)).

%% mongolog_add_rule(+Head, +Body) is semidet.
%
% Register a rule that translates into an aggregation pipeline.
% Any non-terminal predicate in Body must have a previously asserted
% rule it can expand into.
% After being asserted, the Head predicate can be referred to in
% calls of kb_call/1.
%
% @param Head The head of a rule.
% @param Body The body of a rule.
%
mongolog_add_rule(Head, Body) :-
	current_reasoner_module(ReasonerModule),
    mongolog_add_rule(Head, Body, ReasonerModule).

mongolog_add_rule(Head, Body, ReasonerModule) :-
	%% get the functor of the predicate
	Head =.. [Functor|Args],
	%% expand goals into terminal symbols
	(	mongolog_expand(Body, Expanded) -> true
	;	log_error_and_fail(mongolog(expansion_failed(Functor)))
	),
	assertz(mongolog_rule(ReasonerModule, Functor, Args, Expanded)),
	%% define the relation with the reasoner
	length(Args, Arity),
	reasoner_define_relation(Functor, Arity).

%% mongolog_drop_rule(+Head) is semidet.
%
% Drop a previously added `mongolog` rule.
% That is, erase its database record such that it can
% not be referred to anymore in rules added after removal.
%
% @param Term A mongolog rule.
%
mongolog_drop_rule(Head) :-
	compound(Head),
	Head =.. [Functor|_],
	current_reasoner_module(ReasonerModule),
	retractall(mongolog_rule(ReasonerModule, Functor, _, _)).

%%
mongolog_rule1(ReasonerModule, Functor, Args, Expanded) :-
    mongolog_rule(RealReasonerModule, Functor, Args, Expanded),
    once((RealReasonerModule==user ; RealReasonerModule==ReasonerModule)).

%%
define_fact_predicate_(Fact, DocumentID) :-
	current_reasoner_module(Module),
	Fact =.. [Functor|Args],
	length(Args, Arity),
	% make sure define_fact_predicate_ was called for the predicate
	(	mongolog_predicate(Functor, Arity, _, user, _, _) -> true
	;   mongolog_predicate(Functor, Arity, _, Module, _, _) -> true
	;	mongolog_add_predicate(Functor/Arity, DocumentID, [])
	).

%% mongolog_consult(+URL) is semidet.
%
% Load facts and rules from a file into mongolog.
%
mongolog_consult(URL) :-
	mongolog_consult(URL, []).

mongolog_consult(URL, _Options) :-
    % skip consultation if the file was consulted already for the current reasoner.
    % TODO: handle the case that only a subset of predicates was imported before,
    %  i.e. via use_module(FileSpec, ImportList). Currently ImportList is ignored and
    %  all predicates are loaded in any case.
	current_reasoner_module(Module),
	mongolog_source_file(URL, RealModule),
	(RealModule==user ; RealModule==Module),
	!.

mongolog_consult(URL, Options) :-
    log_debug(mongolog(consult(URL,Options))),
    DefaultOpts = [file_type(prolog)],
    (   option(relative_to(CurrentDirectory), Options)
    ->  absolute_file_name(URL, AbsolutePath, [relative_to(CurrentDirectory)|DefaultOpts])
    ;   absolute_file_name(URL, AbsolutePath, DefaultOpts)
    ),
	sw_url_version(AbsolutePath, FileVersion),
	mongolog_consult1(AbsolutePath, FileVersion),
	% remember that the file was consulted
	current_reasoner_module(Module),
	assertz(mongolog_source_file(URL,Module)).

mongolog_consult1(URL, URLVersion) :-
	file_directory_name(URL, FileDirectory),
    % lookup currently stored version of URL
	once((mongolog_stored_source(URL, CurrentVersion, DocumentID, CurrentPredicates) ;
	      CurrentVersion=none)),
	% create entry in 'mongolog_sources' if none exists
	(	CurrentVersion \== none -> true
	;	mongolog_stored_source_create(URL, URLVersion, [], DocumentID)
	),
	% delete facts of older file versions
	(	CurrentVersion == none -> true
	;   CurrentVersion == URLVersion -> true
	;   mongolog_stored_source_drop(DocumentID, CurrentPredicates)
	),
	% read file, assert facts and rules
	% NOTE: rules are not stored in the database, so the file must
	%  be opened again even though it was loaded before.
	ConsultOpts=[source_id(DocumentID),
	             source_url(URL),
	             relative_to(FileDirectory)],
	setup_call_cleanup(
        open(URL, read, QueryStage),
        (	URLVersion==CurrentVersion
        ->	mongolog_consult2(QueryStage, [reload|ConsultOpts])
        ;	mongolog_consult2(QueryStage, [load|ConsultOpts])
        ),
        close(QueryStage)
    ),
	% update version, and array of loaded predicates
	(	URLVersion==CurrentVersion -> true
	;	mongolog_stored_source_update(DocumentID, URL, URLVersion)
	).

mongolog_consult2(QueryStage, Options) :-
	read(QueryStage, Term),
	(	Term==end_of_file -> true
	;	(	mongolog_consult3(Term, Options),
			mongolog_consult2(QueryStage, Options)
		)
	).

% consult a directive
mongolog_consult3((:- use_module(FileSpec)), Options) :-
    !,
    option(relative_to(FileDirectory), Options),
    mongolog_consult(FileSpec, [relative_to(FileDirectory)]).

mongolog_consult3((:- use_module(FileSpec, ImportList)), Options) :-
    !,
    option(relative_to(FileDirectory), Options),
    mongolog_consult(FileSpec, [relative_to(FileDirectory), imports(ImportList)]).

mongolog_consult3((:- module(_Module, _PublicList)), _) :-
    % NOTE: mongolog currently does not support a notion of modules
    !.

mongolog_consult3((:- Goal), _) :-
	!, log_warning(mongolog(unknown_directive(Goal))).

% consult a rule
mongolog_consult3((Head :- Body), Options) :-
	!, mongolog_consult3('?>'(Head,Body), Options).

mongolog_consult3('?+>'(Head,Body), Options) :-
    !, mongolog_consult3('?>'(Head,Body), Options),
       mongolog_consult3('+>'(Head,Body), Options).

mongolog_consult3('?>'(Head,Body), _Options) :-
    !, % "ask" rule
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,_Module,Term),
	% add the rule to the DB backend
	mongolog_add_rule(Term, BodyGlobal).

mongolog_consult3('+>'(Head,Body), _Options) :-
    !, % "tell" rule
	% expand rdf terms Prefix:Local to IRI atom
	rdf_global_term(Head, HeadGlobal),
	rdf_global_term(Body, BodyGlobal),
	strip_module_(HeadGlobal,_Module,Term),
	% rewrite functor
	Term =.. [Functor|Args],
	atom_concat('project_',Functor,Functor0),
	Head0 =.. [Functor0|Args],
	% add the rule to the DB backend
	mongolog_add_rule(Head0, project(BodyGlobal)).

% consult a fact
mongolog_consult3(Fact, Options) :-
	strip_module_(Fact, _Module, Stripped),
	mongolog_consult_fact(Stripped, Options).

%%
mongolog_consult_fact(Fact, Options) :-
	option(load, Options),!,
	option(source_id(DocumentID), Options, user),
	% assert that the asserted predicate is a currently defined predicate
	define_fact_predicate_(Fact, DocumentID),
	% translate into json document
	mongolog_predicate_document(Fact, PredicateDoc),
	% store document
	mongolog_predicate_collection(Fact, DB, PredicateCollection),
	mng_store(DB, PredicateCollection,
	    [['source',string(DocumentID)] | PredicateDoc]).

mongolog_consult_fact(Fact, Options) :-
	% option(reload, Options),
	option(source_id(DocumentID), Options, user),
	% assert that the asserted predicate is a currently defined predicate
	define_fact_predicate_(Fact, DocumentID).

%% The version/last modification time of a loaded file
mongolog_stored_source(URL, Version, DocumentID, Predicates) :-
	mongolog_get_db(DB, Coll, 'mongolog_sources'),
	once(mng_find(DB, Coll, [[url, string(URL)]], Doc)),
	mng_get_dict('_id', Doc, id(DocumentID)),
	mng_get_dict(predicates, Doc, array(Predicates)),
	mng_get_dict(version, Doc, string(Version)).

%%
mongolog_stored_source_create(URL, URLVersion, Collections, DocumentID) :-
	mongolog_get_db(DB, SourcesCollection, 'mongolog_sources'),
    mng_store(DB, SourcesCollection,  [
        ['url',string(URL)],
        ['version',string(URLVersion)],
        ['predicates',array(Collections)]
    ]),
    mongolog_stored_source(URL, _, DocumentID, _).

%%
mongolog_stored_source_update(DocumentID, URL, URLVersion) :-
	mongolog_get_db(DB, SourcesCollection, 'mongolog_sources'),
    % lookup asserted predicate indicators
    findall([[functor,string(Functor)], [arity,int(Arity)]],
        mongolog_predicate(Functor, Arity, _, _, DocumentID, _),
        SourcePredicates),
    mng_update(DB, SourcesCollection, [['_id',id(DocumentID)]], [
        ['url',string(URL)],
        ['version',string(URLVersion)],
        ['predicates', array(SourcePredicates)]
    ]).

%%
mongolog_stored_source_drop(DocumentID, CurrentPredicates) :-
    forall(member(CurrentPred, CurrentPredicates),
        (   memberchk('-'(functor, string(CurrentFunctor)), CurrentPred),
            mongolog_get_db(DB, CurrentFunctorCollection, CurrentFunctor),
            mng_remove(DB, CurrentFunctorCollection, [['source', string(DocumentID)]])
        )).

%%
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).

%% mongolog_call(+Goal) is nondet.
%
% Same as mongolog_call/2 with empty options list.
%
% @param Goal A compound term expanding into an aggregation pipeline
%
mongolog_call(Goal) :-
	query_scope_now(QScope),
	mongolog_call(Goal,[query_scope(QScope)]).

%% mongolog_call(+Goal, +Options) is nondet.
%
% Call Goal by translating it into an aggregation pipeline.
%
% @param Goal A compound term expanding into an aggregation pipeline
% @param Options Additional options
%
mongolog_call(current_predicate(Predicate), _Ctx) :-
	!, mongolog_current_predicate(Predicate).

mongolog_call(consult(File), _Ctx) :-
    !,
    (   var(File)
    ->  throw(error(instantiation_error(File), mongolog_call(consult(File))))
    ;   mongolog_consult(File)
    ).

mongolog_call(Goal, ContextIn) :-
	% Add all toplevel variables to context.
	% note that this is important to avoid that Prolog garbage collects the variables!
	term_keys_variables_(Goal, GlobalVars),
	merge_options([global_vars(GlobalVars)], ContextIn, Context0),
	% retrieve scope of inferred facts and instantiation of inferred predicates if requested
	findall([UserVarKey,_], (
	    ( option(solution_scope(_),ContextIn), UserVarKey='v_scope')
	;   ( option(predicates(_),ContextIn),     UserVarKey='v_predicates')
	),  UserVars0),
	(   UserVars0==[] -> Context = Context0
	;   merge_options([user_vars(UserVars0)], Context0, Context)
	),
	% get the pipeline document
	(   option(predicates(_),ContextIn) -> (
	        % add trace_predicate/1 calls where appropriate
	        expand_instantiations(Goal, GoalWithInstantiations),
	        reasoner:expand_rdf_predicates(GoalWithInstantiations, RDFExpanded),
	        mongolog_compile(RDFExpanded, pipeline(Doc0,Vars), Context),
	        Doc=[['$set',['v_predicates',array([])]] | Doc0]
	    )
	;   (   reasoner:expand_rdf_predicates(Goal, RDFExpanded),
	        mongolog_compile(RDFExpanded, pipeline(Doc,Vars), Context)
	    )
	),
    %
	option(user_vars(UserVars), Context, []),
	option(global_vars(GlobalVars), Context, []),
	append(Vars, UserVars, Vars1),
	append(Vars1, GlobalVars, Vars2),
	list_to_set(Vars2,Vars3),

	% run the pipeline
	query_1(Doc, Vars3),
	%
	(   option(solution_scope(FScope), Context)
	->  format_solution_scope_(Context, FScope)
	;   true
    ),
	(   (option(predicates(Predicates), Context),option(user_vars(UserVars), Context))
	->  memberchk(['v_predicates',Predicates], UserVars)
	;   true
    ).

%%
format_solution_scope_(Context, _{
		time: range(Since,Until),
		uncertain: IsUncertain
}) :-
    option(user_vars(UserVars), Context),
    memberchk(['v_scope',VScope], UserVars),
    % handle time interval
    get_dict(time, VScope, _{
        since: SinceTyped,
        until: UntilTyped
    }),
	mng_strip_type(SinceTyped, _, Since),
	mng_strip_type(UntilTyped, _, Until),
	% handle uncertainty flag
	once(( get_dict(uncertain, VScope, bool(IsUncertain)) ; IsUncertain=false )).

%%
term_keys_variables_(Goal, GoalVars) :-
    term_variables(Goal, Vars),
    term_keys_variables_1(Vars, GoalVars).
term_keys_variables_1([], []) :- !.
term_keys_variables_1([X|Xs], [[Key,X]|Ys]) :-
    term_to_atom(X,Atom),
    atom_concat('v',Atom,Key),
    term_keys_variables_1(Xs, Ys).

%%
query_1(Pipeline, Vars) :-
	% get DB for cursor creation. use collection with just a
	% single document as starting point.
	mongolog_one_db(DB, Coll),
	% run the query
	setup_call_cleanup(
		% setup: create a query cursor
		mng_cursor_create(DB, Coll, Cursor),
		% call: find matching document
		(	mng_cursor_aggregate(Cursor, ['pipeline',array(Pipeline)]),
			query_2(Cursor, Vars)
		),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
query_2(Cursor, Vars) :-
	mng_cursor_materialize(Cursor, Result),
	unify_(Result, Vars),
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
assert_documents1(array(Docs), Coll) :-
	mongolog_get_db(_DB, Coll, 'triples'),!,
	current_reasoner_manager(ReasonerManager),
	current_reasoner_module(ReasonerModule),

	% TODO: rather provide array of triples to cpp instead of
	%       calling pl_assert_triple_cpp9 for each.
	forall(member(StatementData, Docs), (
		dict_pairs(TripleDict,_,StatementData),
		get_dict(s, TripleDict, string(Subject)),
		get_dict(p, TripleDict, string(Predicate)),
		get_dict(graph, TripleDict, string(Graph)),

		get_dict(o, TripleDict, ObjectTerm),
		mng_strip_type(ObjectTerm, _, Object),

		ignore(get_dict(scope, TripleDict, [time-[(since)-double(Since),(until)-double(Until)]])),
		once(
		  ( Since==0.0 -> Begin=_ ; Begin = Since )),
		once(
		  ( Until=='Infinity' -> End=_
		  ; Until=='inf' -> End=_
		  ; End=Until
		  )),
		mng_assert_triple_cpp(
				ReasonerManager,
				ReasonerModule,
				Subject, Predicate, Object,
				Graph, Begin, End, _)
	)).
assert_documents1(array(Docs), Key) :-
	% NOTE: the mongo client currently returns documents as pairs A-B instead of
	%       [A,B] which is required as input.
	% TODO: make mongo return docs in a format that it accepts as input.
	maplist(format_doc, Docs, Docs0),
	maplist(bulk_operation, Docs0, BulkOperations),
	mongolog_get_db(DB, Coll, Key),
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
% unify variables.
%
unify_(Result, Vars) :- unify_0(Result, Vars, Vars).

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
	!,
	mng_strip_type(TypedValue, _, Value),
	% ignore if value in the document is a variable
	(	Value=_{ type: string(var), value: _ }
	% try to unify
	;	Term=Value
	% special case for number comparison, e.g. `5.0 =:= 5`
	;	(	number(Term),
			number(Value),
			Term=:=Value
		)
	).
unify_grounded(_, [_, _]) :- !.

unify_1(_, _, ['_id', _]).

unify_1(Doc, Vars, [VarKey, Val]) :-
	mng_get_dict(VarKey, Doc, TypedValue),
	unify_2(TypedValue, Vars, Val).

%%
unify_2(string(In), _Vars, X) :-
	% handle case that variable is wrapped in term/1.
	% if this is the case then convert input string to term.
	nonvar(X),
	X=term(Out),!,
	atom_to_term_(In, Out).

unify_2(array(In), Vars, Out) :-
	% a variable was instantiated to a list
	!,
	unify_array(In, Vars, Out).

unify_2([K-V|Rest], Vars, Out) :-
	!,
	dict_pairs(Dict,_,[K-V|Rest]),
	unify_2(Dict, Vars, Out).

unify_2(_{ type: string(var), value: string(VarKey) }, Vars, Out) :-
	% a variable was not instantiated
	!,
	memberchk([VarKey, VarVal], Vars),
	Out = VarVal. 

unify_2(_{ type: string(compound), value: Val }, Vars, Out) :-
	% a variable was instantiated to a compound term
	!,
	unify_2(Val, Vars, Out).

unify_2(_{ functor: string(Functor), args: Args }, Vars, Out) :-
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
atom_to_term_(Atom, Term) :-
	% try converting atom stored in DB to a Prolog term
	catch(term_to_atom(Term,Atom), _, fail),
	!.
atom_to_term_(Atom, _) :-
	throw(error(conversion_error(atom_to_term(Atom)))).

%%
mongolog_assert(triple(S,P,O)) :-
	\+ ground([S,P,O]), !,
	throw(error(instantiation_error, assert(triple(S,P,O)))).

mongolog_assert(triple(S,P,term(Compound))) :-
	compound(Compound), !,
	catch(term_to_atom(Compound,O), _, fail),
	mongolog_assert(triple(S,P,O)).

mongolog_assert(triple(S,P,O)) :-
	!,
	current_reasoner_manager(ReasonerManager),
	current_reasoner_module(ReasonerModule),
	sw_default_graph(G),
	mng_assert_triple_cpp(ReasonerManager, ReasonerModule, S, P, O, G, _, _, _).

mongolog_assert(Term) :-
	throw(error(mongolog_error, assert(Term))).

mongolog_assert(Fact) :-
	mongolog_universal_scope(QScope),
	mongolog_call(assert(Fact),[query_scope(QScope)]).

mongolog_assert(triple(S,P,O), Scope) :-
	option(query_scope(QS), Scope),
	mongolog_time_scope(QS, SinceValue0, UntilValue0),
	mng_strip_type(SinceValue0, _, SinceValue),
	mng_strip_type(UntilValue0, _, UntilValue),
	current_reasoner_manager(ReasonerManager),
	current_reasoner_module(ReasonerModule),
	sw_default_graph(G),
	% TODO: include other context parameter
	mng_assert_triple_cpp(ReasonerManager, ReasonerModule,
			S, P, O, G,
			SinceValue, UntilValue, _).

%%
mongolog_project(Fact) :-
	mongolog_universal_scope(QScope),
	mongolog_call(project(Fact),[query_scope(QScope)]).

%% mongolog_retract(+Statement) is nondet.
%
% Same as mongolog_retract/2 with universal scope.
%
% @param Statement a statement term.
%
mongolog_retract(Statement) :-
	mongolog_retract(Statement, dict{}, []).

%% mongolog_retract(+Statement, +Scope) is nondet.
%
% Same as mongolog_retract/3 with empty options list.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
mongolog_retract(Statement, Scope) :-
	mongolog_retract(Statement, Scope, []).

%% kb_unproject(+Statement, +Scope, +Options) is semidet.
%
% Unproject that some statement is true.
% Statement must be a term triple/3.
% It can also be a list of such terms.
% Scope is the scope of the statement to unproject. Options include:
%
%     - graph(GraphName)
%     Determines the named graph this query is restricted to. Note that graphs are organized hierarchically. Default is user.
%
% Any remaining options are passed to the querying backends that are invoked.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
% @param Options list of options.
%
mongolog_retract(_, _, _) :-
	mongolog_is_readonly,
	!.

mongolog_retract(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	forall(member(Statement, Statements),
	       mongolog_retract(Statement, Scope, Options)).

mongolog_retract(triple(S,P,O), Scope, Options) :-
	% TODO: support retraction of other predicates
	% ensure there is a graph option
	set_graph_option(Options, Options0),
	% append scope to options
	merge_options([query_scope(Scope)], Options0, Options1),
	% get the query document
	mng_triple_doc(triple(S,P,O), Doc, Options1),
	% run a remove query
	mongolog_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, Doc).

%%
set_graph_option(Options, Options) :-
	option(graph(_), Options),
	!.
set_graph_option(Options, Merged) :-
	sw_default_graph(DG),
	merge_options([graph(DG)], Options, Merged).

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
	mongolog_expand(Term, Expanded),
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
	% and finally compile expanded terms
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
		VarDocs0
	),
	% make sure there are no duplicate entries as these would cause
	% compilation failure in a single $set!
	list_to_set(VarDocs0,VarDocs),
	(	VarDocs=[] -> Pipeline=Doc
	;	Pipeline=[['$set', VarDocs]|Doc]
	).

%%
step_compile(Step, Ctx, Doc, StepVars) :-
	step_compile(Step, Ctx, Doc),
	step_vars(Step, Ctx, StepVars).

%% ask(:Goal)
% Call Goal in ask mode.
%
step_compile(ask(Goal), Ctx, Doc, StepVars) :-
	mongolog:step_compile(call(Goal), Ctx, Doc, StepVars).

%%
% pragma(Goal) is evaluated compile-time by calling
% the Goal. This is usually done to unify variables
% used in the aggregation pipeline from the compile context.
%
step_compile(pragma(Goal), _, [], StepVars) :-
	% ignore vars referred to in pragma as these are handled compile-time.
	% only the ones also referred to in parts of the query are added to the document.
	StepVars=[],
	call(Goal).

step_compile(stepvars(_), _Ctx, []) :- true.

step_compile(trace_predicate(Predicate), Ctx,
	[['$set', ['v_predicates', ['$concatArrays', array([
		string('$v_predicates'),
		array([ Predicate0 ])
	])]]]]) :-
	var_key_or_val(Predicate, Ctx, Predicate0).

step_command(user,ask).
step_command(user,pragma).
step_command(user,stepvars).
step_command(user,trace_predicate).

%%
match_equals(X, Exp, ['$match', ['$expr', ['$eq', array([X,Exp])]]]).

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
	%			let: { "tests": "$tests"},
	%			pipeline: [{'$set': {"key": "$$tests"}}]
	%		}}])
	%
	findall([Y,string(Y0)],
		(	member([Y,_], InnerVars),
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
	% within a disjunction VV provides mapping between
	% original and copied variables (see control.pl)
	option(orig_vars(VOs), Context, []),
	option(copy_vars(VCs), Context, []),
	% join collection with single document
	mongolog_one_db(_DB, Coll),
	% generate inner pipeline
	compile_terms(Terminals, Pipeline,
		OuterVars->_InnerVars,
		StepVars0, Context),
	% get list of variables whose copies have received a grounding
	% in compile_terms, as these need some special handling
	% to avoid that the original remains ungrounded.
	% GroundVars0: key-original variable mapping
	% GroundVars1: key-grounding mapping
	grounded_vars(Context, [VOs,VCs], GroundVars0, GroundVars1),
	% add variables that have received a grounding in compile_terms
	% to StepVars
	append(GroundVars0, StepVars0, StepVars1),
	list_to_set(StepVars1, StepVars),
	% finally also add user supplied variables to the list
	(	option(additional_vars(AddVars), Context)
	->	append(AddVars, StepVars, StepVars2)
	;	StepVars2 = StepVars
	),
	% pass variables from outer goal to inner if they are referred to
	% in the inner goal.
	lookup_let_doc(StepVars2, LetDoc),
	% set all let variables so that they can be accessed
	% without aggregate operators in Pipeline
	lookup_set_vars(StepVars2, SetVars),
	% compose inner pipeline
	(	SetVars=[] -> Prefix0=Prefix
	;	Prefix0=[['$set', SetVars] | Prefix]
	),
	append(Prefix0,Pipeline,Pipeline0),
	% $set compile-time grounded vars for later unification.
	% this is needed because different branches cannot ground the same
	% variable to different values compile-time.
	% hence the values need to be assigned within the query.
	(	GroundVars1=[] -> Suffix0=Suffix
	;	Suffix0=[['$set', GroundVars1] | Suffix]
	),
	append(Pipeline0,Suffix0,Pipeline1).

% yield list of variables whose copies have received a grounding
% VO: original variable
% VC: copied variable
grounded_vars(Ctx,[VOs,VCs],Xs,Ys) :-
	grounded_vars(Ctx,VOs,VCs,Xs,Ys).
grounded_vars(_,[],[],[],[]) :- !.
grounded_vars(Ctx,
		[[Key,VO]|VOs],
		[[Key,VC]|VCs],
		[[Key,VO]|Xs],
		[[Key,Val]|Ys]) :-
	nonvar(VC),
	\+ is_dict(VC),
	!,
	var_key_or_val(VC, Ctx, Val),
	grounded_vars(Ctx,VOs,VCs,Xs,Ys).
grounded_vars(Ctx,[_|VOs],[_|VCs],Xs,Ys) :-
	grounded_vars(Ctx,VOs,VCs,Xs,Ys).

%grounded_vars([],_,[],[]) :- !.
%grounded_vars([VO-VC|VV],Ctx,[[Key,VO]|Xs],[[Key,Val]|Ys]) :-
%	nonvar(VC),
%	\+ is_dict(VC),
%	!,
%	var_key(VO, Ctx, Key),
%	var_key_or_val(VC, Ctx, Val),
%	grounded_vars(VV,Ctx,Xs,Ys).
%grounded_vars([_|VV],Ctx,Xs,Ys) :-
%	grounded_vars(VV,Ctx,Xs,Ys).

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

%%
add_assertion_var(StepVars, [['g_assertions',_]|StepVars]).

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
    option(query_scope(Scope), Ctx),
    context_var_key(Scope, Key),
    once((
        (option(outer_vars(Vars), Ctx) ; option(disj_vars(Vars), Ctx)),
        member([Key,ReferredVar],Vars)
    )).

%%
context_var_key(Dict, CtxVarKey) :-
	is_dict(Dict),!,
	get_dict(_Key, Dict, Val),
	context_var_key(Val, CtxVarKey).

context_var_key(Val, CtxVarKey) :-
	% NOTE: vars should be resolved to keys in the scope already.
	%       e.g. `Since == =<(string($v_235472))`
	mng_strip(Val, _, string, Stripped),
	atom(Stripped),
	atom_concat('$', CtxVarKey, Stripped).

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
	% TODO: can this be done better than iterating over all variables?
	%		- i.e. by testing if some variable is element of a list
	%		- member/2 cannot be used as it would unify each array element
	(	option(global_vars(Vars), Ctx)
	;	option(outer_vars(Vars), Ctx)
	;	option(step_vars(Vars), Ctx)
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
		), L).

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
	forall(member(Arg,Args),
		(	is_instantiated(Arg, Ctx) -> true
		;	throw(error(instantiation_error))
		)).

is_instantiated(Arg, Ctx) :-
	mng_strip_variable(Arg, Arg0),
	term_variables(Arg0, Vars),
	forall(member(Var, Vars), is_referenced(Var, Ctx)).

		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%% stores files that were loaded via Prolog's consult
:- dynamic expanded_source_file/2.

%%
% Term expansion for *querying* rules using the (?>) operator.
% The body is rewritten such that mongolog_call is called instead
% with body as argument.
% This is mainly used for Prolog files consulted during initialization,
% runtime-consult goes usually via mongolog_call. The difference
% is that here also regular Prolog rules are generated, while
% mongolog_call(consult(File)) only registers the rule with mongolog
% without generating a Prolog rule.
%
user:term_expansion((?>(Head,Body)), Export) :-
    prolog_load_context(source, SourceURL),
	current_reasoner_module(ReasonerModule),
    % note: expansion happens only the first time the file is loaded e.g. via use_module.
    %   a consequitive loading does not trigger expansion again but only makes the predicates
    %   created here available in the module that loads the file again.
    % FIXME: behavior described above may cause problems in case file is loaded multiple times with different reasoner contexts
	expand_ask_rule_(SourceURL, ReasonerModule, Head, Body, Export).

%%
% Term expansion for *project* rules using the (+>) operator.
% The rules are only asserted into mongo DB and expanded into
% empty list.
%
user:term_expansion((+>(Head,Body)), Export) :-
    prolog_load_context(source, SourceURL),
	current_reasoner_module(ReasonerModule),
	expand_tell_rule_(SourceURL, ReasonerModule, Head, Body, Export).

%%
% Term expansion for *query+project* rules using the (?+>) operator.
% These are basically clauses that can be used in both contexts.
%
% Consider for example following rule:
%
%     is_event(Entity) ?+>
%       has_type(Entity, dul:'Event').
%
% This is valid because, in this case, has_type/2 has
% clauses for querying and projection.
%
user:term_expansion((?+>(Head,Goal)), X1) :-
	user:term_expansion((?>(Head,Goal)),X1),
	user:term_expansion((+>(Head,Goal)),_X2).

%%
expand_ask_rule_(SourceFile, ReasonerModule, _Head, _Body, []) :-
    is_expanded_(SourceFile, ReasonerModule), !.
expand_ask_rule_(SourceFile, ReasonerModule, Head, Body, Export) :-
    % remember source file was loaded by a reasoner
    expand_assert_source_(SourceFile, ReasonerModule),
	% register the clause with mongolog
    mongolog:mongolog_consult3((?>(Head,Body)), []),
	% expand into regular Prolog rule only once for all clauses
	strip_module_(Head, _RuleModule, Term),
	Term =.. [Functor|Args],
	length(Args, Arity),
    prolog_load_context(module, DstModule),
	(	DstModule:current_predicate(Functor/Arity) -> Export=[]
	;	(   query_scope_now(QScope),
	        length(Args1,Arity),
	        Term1 =.. [Functor|Args1],
		    Export=[(:-(Term1, mongolog:mongolog_call(Term1, [query_scope(QScope)])))]
	    )
	).

%%
expand_tell_rule_(SourceFile, ReasonerModule, _Head, _Body, []) :-
    is_expanded_(SourceFile, ReasonerModule), !.
expand_tell_rule_(SourceFile, ReasonerModule, Head, Body, []) :-
    expand_assert_source_(SourceFile, ReasonerModule),
    mongolog:mongolog_consult3((+>(Head,Body)), []).

%%
expand_assert_source_(SourceFile, ReasonerModule) :-
    mongolog_source_file(SourceFile, ReasonerModule),!.
expand_assert_source_(SourceFile, ReasonerModule) :-
    assertz(mongolog_source_file(SourceFile, ReasonerModule)),
    assertz(expanded_source_file(SourceFile, ReasonerModule)).

%%
is_expanded_(SourceFile, ReasonerModule) :-
    % proceed if the source file is being expanded
    \+ expanded_source_file(SourceFile, ReasonerModule),
    % test if the file was loaded before
	mongolog_source_file(SourceFile, RealModule),
	(RealModule==user ; RealModule==ReasonerModule),
	!.

%%
% explicitely do not expand some of the meta predicates
expand_instantiations((\+ Goal), (\+ Goal)) :- !.
expand_instantiations(not(Goal), not(Goal)) :- !.
expand_instantiations(forall(Cond,Action), forall(Cond,Action)) :- !.
% meta predicates like `,/2`, `call/1`, `once/1`, `->\2` that receive a goal as an argument,
% and where the goal argument can be expanded.
% TODO: findall-family of predicates and instantiations in their goal argument
expand_instantiations(Head, Expanded) :-
	user:predicate_property(Head, meta_predicate(MetaSpecifier)), !,
	Head =.. [HeadFunctor|Args],
	MetaSpecifier =.. [_MetaFunctor|Modes],
	expand_meta_instantiations(Args, Modes, ExpandedArgs),
	Expanded =..  [HeadFunctor|ExpandedArgs].
% user defined relations
expand_instantiations(Head, (Head, trace_predicate(Head))) :-
    Head =.. [Functor|Args], length(Args,Arity),
    mongolog_current_predicate(Functor/Arity, idb_relation), !.
% builtins
expand_instantiations(Head, Head).

%%
expand_meta_instantiations([], [], []) :- !.
expand_meta_instantiations([ArgN|Args], [ModeN|Modes], [Next|Rest]) :-
	number(ModeN),!, % higher-order argument
	expand_instantiations(ArgN, Next),
    expand_meta_instantiations(Args, Modes, Rest).
expand_meta_instantiations([ArgN|Args], [_|Modes], [ArgN|Rest]) :-
    expand_meta_instantiations(Args, Modes, Rest).

%% mongolog_expand(+Term, -Expanded) is det.
%
% Translate a goal into a sequence of terminal commands.
% Terminal commands are the core predicates supported in queries
% such as arithmetic and comparison predicates.
% Rules, on the other hand, are "flattened" during term expansion,
% and translated to a sequence of these terminal commands.
%
% @param Term A compound term, or a list of terms.
% @param Expanded Sequence of terminal commands
%
mongolog_expand(Goal, Goal) :-
	% goals maybe not known during expansion, i.e. in case of
	% higher-level predicates receiving a goal as an argument.
	% these var goals need to be expanded compile-time
	% (call-time is not possible)
	var(Goal), !.

mongolog_expand(Goal, Expanded) :-
	% NOTE: do not use is_list/1 here, it cannot handle list that have not
	%       been completely resolved as in `[a|_]`.
	%       Here we check just the head of the list.
	\+ has_list_head(Goal), !,
	comma_list(Goal, Terms),
	mongolog_expand(Terms, Expanded).

mongolog_expand(Terms, Expanded) :-
	has_list_head(Terms), !,
	catch(expand_term_0(Terms, Expanded0),
		  Exc,
		  log_error_and_fail(lang(Exc, Terms))),
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
expand_term_0([], []) :- !.
expand_term_0([X|Xs], [X_expanded|Xs_expanded]) :-
	once(expand_term_1(X, X_expanded)),
	% could be that expand-time the list is not fully resolved
	(	var(Xs) -> Xs_expanded=Xs
	;	expand_term_0(Xs, Xs_expanded)
	).

expand_term_1(Goal, Expanded) :-
	% TODO: seems nested terms sometimes not properly flattened, how does it happen?
	is_list(Goal),!,
	expand_term_0(Goal, Expanded).

expand_term_1(Goal, Expanded) :-
	current_reasoner_module(ReasonerModule),
	once(is_step_command(ReasonerModule, Goal)),
	% allow the goal to recursively expand
	(	mongolog:step_expand(Goal, Expanded) -> true
	;	Expanded = Goal
	).

expand_term_1(Goal, Expanded) :-
	% expand the rule head (Goal) into terminal symbols (the rule body)
	(	expand_rule(Goal, Clauses) -> true
	% handle the case that a predicate is referred to that wasn't asserted before
	;	throw(expansion_failed(Goal))
	),
	% wrap different clauses into ';'
	maplist(flatten, Clauses, ClausesFlat),
	semicolon_list(Disjunction, ClausesFlat),
	mongolog_expand(Disjunction, Expanded).

%%
expand_rule(Goal, Terminals) :-
	% ground goals do not require special handling for variables
	% as done in the clause below. So this clause here is simpler.
	ground(Goal),!,
	current_reasoner_module(ReasonerModule),
	% unwrap goal term into functor and arguments.
	Goal =.. [Functor|Args],
	% findall rules with matching functor and arguments
	findall(X, mongolog_rule1(ReasonerModule, Functor, Args, X), TerminalClauses),
	(	TerminalClauses \== []
	->	Terminals = TerminalClauses
	% if TerminalClauses==[] it means that either there is no such rule
	% in which case expand_rule fails, or there is a matching rule, but
	% the arguments cannot be unified with the ones provided in which
	% case expand_rule succeeds with a pipeline [fail] that allways fails.
	;	(	once(mongolog_rule1(ReasonerModule, Functor,_,_)),
			Terminals=[fail]
		)
	).

expand_rule(Goal, Terminals) :-
	current_reasoner_module(ReasonerModule),
	% unwrap goal term into functor and arguments.
	Goal =.. [Functor|Args],
	% find all asserted rules matching the functor
	findall([Args0,Terminals0],
			(	mongolog_rule1(ReasonerModule, Functor, Args0, Terminals0),
				unifiable(Args0, Args, _)
			),  Clauses),
	Clauses \= [],
	expand_rule(Args, Clauses, Terminals).

% prepend pragma call that unifies "child" and "parent" arguments
expand_rule(_, [], []) :- !.
expand_rule(ParentArgs,
		[[ChildArgs,Terminals]|Xs],
		[Expanded|Ys]) :-
	Expanded=[
		% HACK: force that ParentArgs are added to variable map as
		%       following pragma call may make the variables disappear.
		stepvars(ParentArgs),
		pragma(=(ChildArgs,ParentArgs)),
		Terminals
	],
	expand_rule(ParentArgs, Xs, Ys),
	!.

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

%
step_expand(ask(Goal), ask(Expanded)) :-
	mongolog_expand(Goal, Expanded).

% split list at cut operator.
take_until_cut([],[],[]) :- !.
take_until_cut(['!'|Xs],[],['!'|Xs]) :- !.
take_until_cut([X|Xs],[X|Ys],Remaining) :-
	take_until_cut(Xs,Ys,Remaining).

% 
has_list_head([]) :- !.
has_list_head([_|_]).

     /*******************************
     *        SEARCH INDEX          *
     *******************************/

%% setup_collection(+Name, +Indices) is det.
%
% Configure the indices of a named collection.
%
setup_collection(Name, Indices) :-
	assertz(collection_data_(Name, Indices)),
	create_indices(Name, Indices).

%%
create_indices :-
	forall(collection_data_(Name, Indices),
		   create_indices(Name, Indices)).

%% Create indices for fast annotation retrieval.
create_indices(_Name, _Indices) :-
	mongolog_is_readonly,
	!.
create_indices(Name, Indices) :-
	mongolog_get_db(DB, Coll, Name),
	forall(member(Index, Indices),
		   mng_index_create(DB, Coll, Index)).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- rdf_meta(test_call(t,?,t)).

%%
test_call(Goal, Var, Value) :-
	WithSet=(','(assign(Var,Value), Goal)),
	mongolog_call(WithSet).

