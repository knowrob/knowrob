:- module(mng_query,
    [ mng_assert(t),
      mng_ask(t,+,+),
      mng_tell(t,+,+),
      mng_expand/3,
      mng_query_command/1
    ]).
/** <module> Integration of KnowRob language terms with mongo DB.

KnowRob language terms are defined in rules which are translated
into aggregate pipelines in this module.

@author Daniel Beßler
@license BSD
*/

% TODO:
% - *tell* case is missing
% - check fact scope unification
%		- seems missing!
% - look into building list with patterns in findall
%		- shouldn't be hard to do in unififcation step (just need to store pattern with var)
% - cut operator
%		- maybe via lookup with $limit as last step?
% - recursion in rules impossible?
%		- $graphLookup allows limited recursion, what exactly are the limits?
%		- can we make recursion with chains of properties in graphLookup?
% - UNITS
%     -- units are optionally stored in the document
%     -- qudt RDF model defines unit conversion
%     -- holds allows unit conversion, e.g.
%           holds(A,B,kg(X)), holds(C,B,kg(X))
%        best would be if aggregate pipeline performs the conversion.
%        needs arithmetic expression to compute value
% - list commands might be needed
% - string commands might be needed
% - term commands?
% - implement unify operator =/2, i.e. $set if var, $match if ground.
% - include_parents is needed? should be taken into account to yield
%         all elements in p* of triples
%

%% Stores list of terminal terms for each clause. 
:- dynamic mng_query/4.
%% set of registered query commands.
:- dynamic step_command/1.
%% optionally implemented by query commands.
:- multifile step_expand/3.

%% mng_query_command(+Command) is det.
%
% register a command that can be used in KnowRob
% language expressions and which is implemented
% in a mongo query.
% Command is a term used for unification, e.g. `match(_)` for
% the *match* command as it expectes a single argument.
% NOTE: to implement a command several multifile predicates in
% mng_query and mng_compiler must be implemented by a command. 
%
% @Command a command term.
%
mng_query_command(Command) :-
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
mng_ask(Statement, Scope, Options) :-
	Context = [ask, scope(Scope) | Options],
	query_(Statement, Context).

%% mng_tell(+Statement, +Scope, +Options) is semidet.
%
% Run a mongo query to assert that a Statement holds
% within Scope.
%
% @Statement a statement written in KnowRob language
% @Scope the scope of the statement
% @Options query options
%
mng_tell(Statement, Scope, Options) :-
	Context = [ask, scope(Scope) | Options],
	query_(Statement, Context).

%%
query_(Goal, Context) :-
	%% expand goals into terminal symbols
	mng_expand(Goal, Expanded, Context),
	%% get the pipeline document
	mng_compile(Expanded, pipeline(Doc,Vars), Context),
	%% run the pipeline
	query_1(Doc, Vars).

query_1(Pipeline, Vars) :-
	%% get DB for cursor creation
	triple_db(DB, Coll),
	%% run the query
	setup_call_cleanup(
		% setup: create a query cursor
		mng_cursor_create(DB, Coll, Cursor),
		% call: find matching document
		(	mng_cursor_aggregate(Cursor, ['pipeline',array(Pipeline)]),
			mng_cursor_materialize(Cursor, Result),
			unify_(Result, Vars)
		),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
unify_(_, []) :- !.
unify_(Doc, [X|Xs]) :-
	once(unify_1(Doc, X)),
	unify_(Doc, Xs).

unify_1(_, ['_id', _]).
unify_1(Doc, [VarKey, Var]) :-
	mng_get_dict(VarKey, Doc, TypedValue),
	mng_strip_type(TypedValue, _, Value),
	(	Value='null' ;
		Var=Value
	).

%% mng_assert(+Rule) is semidet.
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
mng_assert(Head ?> Body) :-
	mng_assert1(Head, Body, ask).

mng_assert(Head +> Body) :-
	mng_assert1(Head, Body, tell).

mng_assert1(Head, Body, Context) :-
	%% get the functor of the predicate
	Head =.. [Functor|Args],
	%% expand goals into terminal symbols
	(	mng_expand(Body, Expanded, Context)
	->	true
	;	(	log_error(mongo(assertion_failed(Functor,Body))),
			fail
		)
	),
	%% store expanded query
	assertz(mng_query(Functor, Args, Expanded, Context)).

%% mng_expand(+Goal, -Expanded, +Context) is det.
%
% Translates a KnowRob langauge term into a sequence
% of commands that can be executed by mongo DB.
%
% @Goal a KnowRob language term
% @Expanded sequence of commands
% @Context 'ask' or 'tell'
%
mng_expand(Goal, Expanded, Context) :-
	catch(
		expand_term_0(Goal, Expanded, Context),
		expansion_failed(FailedGoal),
		(	log_error(mongo(expansion_failed(FailedGoal, Goal))),
			fail
		)
	).

expand_term_0(Goal, Expanded, Context) :-
	comma_list(Goal, Terms),
	findall(X,
		(	member(Term, Terms),
			expand_term_1(Term, X, Context)
		),
		Expanded
	).

%% try expanding disjuntion, else call expand_term_2
expand_term_1(Goal, Expanded, Context) :-
	semicolon_list(Goal, Terms),
	(	Terms=[_]
	->	expand_term_2(Goal, Expanded, Context)
	;	expand_term_2(facet(Terms), Expanded, Context)
	).

%% strip all modifiers from the goal and call expand_term_3
expand_term_2(Goal, step(Expanded, ModifierList), Context) :-
	% TODO: print warning if modifiers are used that are not supported
	strip_all_modifier(Goal, Stripped, ModifierList),
	expand_term_3(Stripped, Expanded, Context).

%% finally expand rules that were asserted before
expand_term_3(Goal, Expanded, Context) :-
	step_command(Goal, _), !,
	(	step_expand(Goal, Expanded, Context)
	->	true
	;	Expanded = Goal
	).

expand_term_3(Goal, Expanded, Context) :-
	% find all asserted rules matching the functor and args
	Goal =.. [Functor|Args],
	findall(Terminals,
		mng_query(Functor, Args, Terminals, Context),
		TerminalsList),
	% handle the case that a predicate is referred to that wasn't
	% asserted before
	(	TerminalsList=[]
	->	throw(expansion_failed(Goal))
	;	true
	),
	% need to wrap in facet/1 to indicate that there are multiple clauses.
	% the case of multiple clauses is handled using the $facet command.
	(	TerminalsList=[List]
	->	member(Expanded,List)
	;	expand_term_2(facet(TerminalsList), Expanded, Context)
	).

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

