:- module(mongolog,
	[ mongolog_call(t),
	  mongolog_call(t,+),
	  mongolog_assert(t),
	  mongolog_retract(t),        % +Statement
	  mongolog_retract(t,t),      % +Statement, +Scope
	  mongolog_retract(t,t,t),    % +Statement, +Scope, +Options
	  mongolog_add_rule(t,t),
	  mongolog_drop_rule(t),
	  mongolog_expand(t,-),
	  is_mongolog_predicate(+),
	  setup_collection/2
	]).
/** <module> Compiling goals into aggregation pipelines.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1, rdf_global_term/2 ]).
:- use_module(library('scope')).
:- use_module(library('mongolog/subgraph'), [ set_graph_option/2 ]).
:- use_module(library('mongodb/client')).
:- use_module(library('logging')).


%% set of registered query commands.
:- dynamic step_command/1.
% optionally implemented by query commands.
:- multifile step_expand/2.
%% implemented by query commands to compile query documents
:- multifile step_compile/3, step_compile/4.
%%
:- dynamic mongolog_rule/3.

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


%% is_mongolog_predicate(+PredicateIndicator) is semidet.
%
% True if PredicateIndicator corresponds to a known mongolog predicate.
%
is_mongolog_predicate((/(Functor,_Arity))) :-
	!, step_command(Functor).
	
is_mongolog_predicate(Goal) :-
	compound(Goal),!,
	Goal =.. [Functor|_Args],
	step_command(Functor).
	
is_mongolog_predicate(Functor) :-
	atom(Functor),!,
	step_command(Functor).

%% mongolog_consult(+URL) is semidet.
%
% Load facts and rules from a file into mongolog.
%
mongolog_consult(URL) :-
	absolute_file_name(URL, AbsolutePath),
	ontology_url_version(AbsolutePath, FileVersion),
	mongolog_consult1(AbsolutePath, FileVersion).

mongolog_consult1(URL, URLVersion) :-
	once((get_current_file_version(URL, CurrentVersion) ; CurrentVersion=none)),
	open(URL, read, Stream),
	(	URLVersion==CurrentVersion
	->	mongolog_consult2(Stream, reload)
	;	mongolog_consult2(Stream, load)
	),
	close(Stream),
	% remember that the file was loaded
	% TODO: it seems more elegant if data and file kept in same collection
	mng_get_db(DB, Coll, 'mongolog_files'),
	(	CurrentVersion==none
	->	mng_store(DB, Coll,  [['url',string(URL)], ['version',string(URLVersion)]])
	;	URLVersion==CurrentVersion
	->	true
	;	mng_update(DB, Coll, [['url',string(URL)]], [['version',string(URLVersion)]])
	).

mongolog_consult2(Stream, Mode) :-
	read(Stream, Term),
	(	Term==end_of_file -> true
	;	(	mongolog_consult3(Term, Mode),
			mongolog_consult2(Stream, Mode)
		)
	).

mongolog_consult3(':-'(Head, Body), _) :-
	!,
	mongolog_add_rule(Head, Body).

mongolog_consult3(':-'(_Goal), _) :-
	% TODO: support directives
	log_warning('mongolog does not support consulting directives'),!.

mongolog_consult3(Fact, load) :-
	mongolog_add_predicate(Fact),
	% TODO: bulk assert facts from consulted files
	mongolog_call(assert(Fact)).

mongolog_consult3(Fact, reload) :-
	mongolog_add_predicate(Fact).

%%
mongolog_add_predicate(Fact) :-
	Fact =.. [Functor|Args],
	% make sure mongolog_add_predicate was called for the predicate
	(	mongolog_predicate(Functor,_,_) -> true
	;	(	length(Args,NumArgs),
			findall(FieldName,(
				between(1,NumArgs,X),
				% just use argument number as name
				atom_number(FieldName,X)
			),Fields),
			mongolog_add_predicate(Functor, Fields, [])
		)
	).

%% The version/last modification time of a loaded file
get_current_file_version(URL, Version) :-
	mng_get_db(DB, Coll, 'mongolog_files'),
	once(mng_find(DB, Coll, [
		['url', string(URL)]
	], Doc)),
	mng_get_dict(version, Doc, string(Version)).
	

%% mongolog_call(+Goal) is nondet.
%
% Same as mongolog_call/2 with empty options list.
%
% @param Goal A compound term expanding into an aggregation pipeline
%
mongolog_call(Goal) :-
	current_scope(QScope),
	mongolog_call(Goal,[scope(QScope)]).

%% mongolog_call(+Goal, +Options) is nondet.
%
% Call Goal by translating it into an aggregation pipeline.
%
% @param Goal A compound term expanding into an aggregation pipeline
% @param Options Additional options
%
mongolog_call(is_mongolog_predicate(P), _Ctx) :-
	!,
	is_mongolog_predicate(P).

mongolog_call(consult(File), _Ctx) :-
	!,
	mongolog_consult(File).

mongolog_call(Goal, Context) :-
	% get the pipeline document
	mongolog_compile(Goal, pipeline(Doc,Vars), Context),
	%
	option(user_vars(UserVars), Context, []),
	option(global_vars(GlobalVars), Context, []),
	append(Vars, UserVars, Vars1),
	append(Vars1, GlobalVars, Vars2),
	list_to_set(Vars2,Vars3),
	% run the pipeline
	query_1(Doc, Vars3).

%%
mongolog_assert(Fact) :-
	universal_scope(QScope),
	mongolog_call(assert(Fact),[scope(QScope)]).

%% mongolog_retract(+Statement) is nondet.
%
% Same as mongolog_retract/2 with universal scope.
%
% @param Statement a statement term.
%
mongolog_retract(Statement) :-
	wildcard_scope(Scope),
	mongolog_retract(Statement, Scope, []).

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
	setting(mng_client:read_only, true),
	!.

mongolog_retract(Statements, Scope, Options) :-
	is_list(Statements),
	!,
	forall(
		member(Statement, Statements),
		mongolog_retract(Statement, Scope, Options)
	).

mongolog_retract(triple(S,P,O), Scope, Options) :-
	% TODO: support other language terms too
	%	- rather map to kb_call(retractall(Term)) or kb_call(unproject(Term))
	% ensure there is a graph option
	set_graph_option(Options, Options0),
	% append scope to options
	merge_options([scope(Scope)], Options0, Options1),
	% get the query document
	mng_triple_doc(triple(S,P,O), Doc, Options1),
	% run a remove query
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, Doc).

query_1(Pipeline, Vars) :-
	% get DB for cursor creation. use collection with just a
	% single document as starting point.
	mng_one_db(DB, Coll),
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
assert_documents1(array(Docs), Key) :-
	% NOTE: the mongo client currently returns documents as pairs A-B instead of
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
% unify variables.
%
unify_(Result, Vars) :-
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
atom_to_term_(Atom, Term) :-
	% try converting atom stored in DB to a Prolog term
	catch(term_to_atom(Term,Atom), _, fail),
	!.

atom_to_term_(Atom, Term) :-
	% vectors maybe stored space-separated.
	% @deprecated
	atomic_list_concat(Elems, ' ', Atom),
	maplist(atom_number, Elems, Term),
	!.

atom_to_term_(Atom, _) :-
	throw(error(conversion_error(atom_to_term(Atom)))).


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
%step_compile(pragma(Goal), _, []) :-
%	call(Goal).

step_compile(pragma(Goal), _, [], StepVars) :-
	% ignore vars referred to in pragma as these are handled compile-time.
	% only the ones also referred to in parts of the query are added to the document.
	StepVars=[],
	call(Goal).

step_compile(stepvars(_), _, []) :- true.

step_command(ask).
step_command(pragma).
step_command(stepvars).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% QUERY DOCUMENTS
%%%%%%%%%%%%%%%%%%%%%%%

%%
match_equals(X, Exp, ['$match', ['$expr', ['$eq', array([X,Exp])]]]).

%%
match_scope(['$match', ['$expr', ['$lt', array([
				string('$v_scope.time.since'),
				string('$v_scope.time.until')
			])]]]).

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
	mng_one_db(_DB, Coll),
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

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% VARIABLES in queries
%%%%%%%%%%%%%%%%%%%%%%%

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
	% TODO: can this be done better then iterating over all variables?
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


		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

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
	%% get the functor of the predicate
	Head =.. [Functor|Args],
	%% expand goals into terminal symbols
	(	mongolog_expand(Body, Expanded) -> true
	;	log_error_and_fail(lang(assertion_failed(Body), Functor))
	),
	assertz(mongolog_rule(Functor, Args, Expanded)).


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
	retractall(mongolog_rule(Functor, _, _)).

		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

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
	catch(
		expand_term_0(Terms, Expanded0),
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
	once(is_mongolog_predicate(Goal)),
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
	% unwrap goal term into functor and arguments.
	Goal =.. [Functor|Args],
	% findall rules with matching functor and arguments
	findall(X, mongolog_rule(Functor, Args, X), TerminalClauses),
	(	TerminalClauses \== []
	->	Terminals = TerminalClauses
	% if TerminalClauses==[] it means that either there is no such rule
	% in which case expand_rule fails, or there is a matching rule, but
	% the arguments cannot be unified with the ones provided in which
	% case expand_rule succeeds with a pipeline [fail] that allways fails.
	;	(	once(mongolog_rule(Functor,_,_)),
			Terminals=[fail]
		)
	).

expand_rule(Goal, Terminals) :-
	% unwrap goal term into functor and arguments.
	Goal =.. [Functor|Args],
	% find all asserted rules matching the functor
	findall([Args0,Terminals0],
			(	mongolog_rule(Functor, Args0, Terminals0),
				unifiable(Args0, Args, _)
			),
			Clauses),
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
	forall(
		collection_data_(Name, Indices),
		create_indices(Name, Indices)
	).

%% Create indices for fast annotation retrieval.
create_indices(_Name, _Indices) :-
	setting(mng_client:read_only, true),
	!.
create_indices(Name, Indices) :-
	mng_get_db(DB, Coll, Name),
	forall(
		member(Index, Indices),
		mng_index_create(DB, Coll, Index)
	).


     /*******************************
     *    SPECIAL COLLECTIONS       *
     *******************************/

%%
% True if "one" collection has a document.
%
has_one_db :-
	mng_one_db(DB, Collection),
	mng_find(DB, Collection, [], _),
	!.

%%
% Ensure that "one" collection has one document.
%
initialize_one_db :-
	has_one_db, !.

initialize_one_db :-
	mng_one_db(DB, Collection),
	mng_store(DB, Collection, [
		['v_scope', [
			['time', [
				['since',double(0)],
				['until',double('Infinity')]
			]]
		]]
	]).

% make sure collection "one" has a document
:- once((setting(mng_client:read_only, true) ; initialize_one_db)).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- rdf_meta(test_call(t,?,t)).

%%
test_call(Goal, Var, Value) :-
	WithSet=(','(assign(Var,Value), Goal)),
	mongolog_call(WithSet).

