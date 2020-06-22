:- module(lang_query,
    [ ask(t),      % +Statement
      ask(t,t),    % +Statement, +Scope
      tell(t),     % +Statement
      tell(t,t),   % +Statement, +Scope
      update(t),   % +Statement
      update(t,t), % +Statement, +Scope
      is_synchronized(t),  % +Statement
      synchronize(t)       % +Statement
    ]).
/** <module> Main interface predicates for querying the knowledge base.

@author Daniel Beßler
@license BSD
*/

:- op(1100, xfx, user:(?>)).
:- op(1100, xfx, user:(+>)).
:- op(1100, xfx, user:(?+>)).

:- use_module(library('db/scope'),
    [ scope_intersect/3,
      scope_update/3,
      scope_remove/4
    ]).
%:- use_module(library('reasoning/pool'),
    %[ infer/3 ]).

:- multifile tell/2, ask2/2.

%% ask(+Statement) is nondet.
%
% Same as ask/2 with wildcard scope.
%
% @param Statement a statement term.
%
ask(Statement) :-
  current_scope(QScope),
  ask(Statement,[[],QScope]->_).

%% ask(+Statement,+Scope) is nondet.
%
% True if Statement term holds within the requested scope.
% Scope is a term `[Options,QueryScope]->FactScope` where QueryScope
% is the scope requested, and FactScope the actual scope
% of the statement being true.
% Statement can also be a list of statements.
%
% ask/2 is a multifile predicate. Meaning that clauses may be
% decalared in multiple files.
% Specifically, declaring a rule using the ask operator `?>`,
% or the ask-tell operator `?+>` will generate a clause of the ask rule.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
ask(Statement,[Options,QScope]) :-
	select_option(callback(CB),Options,Options1),
	!,
	ask_asynch(Statement,[Options1,QScope],CB).

ask(Statements,QScope->FScope) :-
  is_list(Statements),!,
  ask_all_(Statements,QScope,_->FScope).

ask(Statement,QScope->FScope) :-
  ground(Statement),!,
  % TODO only cut choicepoints in case fact scope is universal!
  %universal_scope(US),
  ask1(Statement,QScope->FScope),
  %( scope_equal(FScope,US) -> ! ; true )
  !.

ask(Statement,Scope) :-
  ask1(Statement,Scope).

ask1(triple(S,P,O),[Options,QScope]->FScope) :-
  !,
  % tripledb retrieval
  tripledb_ask(S,P,O,QScope,FScope,Options).

ask1(Statement,Scope) :-
  % handle complex statements
  ask2(Statement,Scope).

ask1(Statement,Scope) :-
  % try to infer the statement
  reasoning_pool:infer(Statement,_,Scope).

%%
ask_all_([],_QS,FS->FS) :-
  !.
ask_all_([{X}|Xs],QS,FS1->FSn) :-
  !,
  call(X),
  ask_all_(Xs,QS,FS1->FSn).
ask_all_([X|Xs],QS,FS->FSn) :-
  ask(X,QS->FS0),
  % TODO: fact scope should restrict query scope for next query!
  scope_intersect(FS,FS0,FS1),
  ask_all_(Xs,QS,FS1->FSn).

%% tell(+Statement) is nondet.
%
% Same as tell/2 with universal scope.
%
% @param Statement a statement term.
%
tell(Statement) :-
  universal_scope(FScope),
  tell(Statement,[[],FScope]).

%% tell(+Statement,+Scope) is det.
%
% Tell the knowledge base that some statement is true.
% Scope is a term `[Options,FactScope]` where FactScope
% the scope of the statement being true.
% Statement can also be a list of statements.
%
% tell/2 is a multifile predicate. Meaning that clauses may be
% decalared in multiple files.
% Specifically, declaring a rule using the tell operator `+>`,
% or the ask-tell operator `?+>` will generate a clause of the tell rule.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
tell(Statements,Scope) :-
  is_list(Statements),!,
  tell_all(Statements,Scope).

tell(triple(S,P,O),[Options,QScope]) :-
  !,
  ( ask(is_description_of(O,Resource))
  -> true
  ;  Resource=O
  ),
  tripledb_tell(S,P,Resource,QScope,Options).

tell(triple(S,P,O),QScope) :-
  is_dict(QScope),!,
  tell(triple(S,P,O),[[],QScope]).

%%
tell_all([],_) :- !.
tell_all([X|Xs],QScope) :-
  tell(X,QScope),
  tell_all(Xs,QScope).

%% update(+Statement) is nondet.
%
% Same as tell/1 but replaces existing overlapping values.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
update(Statement) :-
	universal_scope(FScope),
	update(Statement,[[],FScope]).

%% update(+Statement,+Scope) is nondet.
%
% Same as tell/2 but replaces existing overlapping values.
%
% @param Statement a statement term.
% @param Scope the scope of the statement.
%
update(Statement,Scope0) :-
	context_update_(Scope0,[options([functional])],Scope1),
	tell(Statement,Scope1).

		 /*******************************
		 *	    ASYNCH ASK	     		*
		 *******************************/

%% A message queue shared between multiple worker threads.
:- message_queue_create(_,[alias(ask_requests)]).
:- message_queue_create(_,[alias(ask_queries)]).

%%
% Maximum number of asynch ask operations running in parallel.
% Threads are created once, then sleep until ask requests are made.
%
num_ask_threads(4).

%%
% Asynchronously call an ask query without blocking until results
% are obtained.
%
ask_asynch(Query,[Options,QScope],Callback) :-
	% priority has influence on which request is handled first in case
	% there are more requests then threads available
	option(priority(Priority),Options,low),
	% extract subject from query (it must be first argument)
	% FIXME: this won't work always, or?
	Query=..[_,Subject|_],
	Msg=request(
		subject(Subject),
		query(Query,[Options,QScope]),
		callback(Callback),
		priority(Priority)
	),
	% send the request message to ask_queries and ask_requests queue.
	% it is removed from ask_requests as soon as a thread picks
	% the query, and it is removed from ask_queries once the thread
	% is done.
	thread_send_message(ask_queries, Msg),
	thread_send_message(ask_requests,Msg).

%%
% A worker thread answering queries.
%
ask_loop :-
	repeat,
	% pop query from queue, this is a blocking call
	get_request(Msg),
	% ask a query
	ask_with_callback(Msg),
	% pop out Msg from ask_queries queue to indicate that
	% query processing has finished
	thread_get_message(ask_queries,Msg),
	% fallback to 'repeat' above
	fail.

%%
ask_with_callback(request(_,query(Query,QScope),callback(Callback),_)) :-
	catch(
		ask_with_callback1(Callback,Query,QScope),
		Exc,
		print_message(warning, query_exception(Query,Exc))
	).
ask_with_callback1(all(Goal),Query,QScope) :-
	% *all* callbacks are called once with the list of all answers.
	!,
	findall([Query,FScope],
		ask(Query,QScope->FScope),
		Solutions
	),
	list_to_set(Solutions,Set),
	call(Goal,Set).
ask_with_callback1(each(Goal),Query,QScope) :-
	% *each* callbacks are called for each individual answer.
	!,
	forall(
		ask(Query,QScope->FScope),
		call(Goal,[Query,FScope])
	).
ask_with_callback1(one(Goal),Query,QScope) :-
	% *one* callbacks are called only once for the first answer obtained,
	% or not at all in case no answer could be found.
	!,
	( ask(Query,QScope->FScope)
	-> call(Goal,[Query,FScope])
	;  true
	).
ask_with_calback1(UnknownTerm,_,_) :-
	throw(error(syntax_error, ask(callback(UnknownTerm)))).

%%
get_request(request(S,Q,C,priority(high))) :-
	% prefer requests with high priority
	thread_get_message(ask_requests,
		request(S,Q,C,priority(high)),
		[timeout(0)]),
	!.
get_request(Msg) :-
	% pick any request
	thread_get_message(ask_requests,Msg).

%%
ask_threads_create :-
	num_ask_threads(Count),
	forall(
		between(1,Count,_),
		thread_create(ask_loop,_)
	).
% create threads when this module is consulted
:- ask_threads_create.

%%
% True for queries that are not scheduled to be answered by
% an asynch ask operation, nor are currently being processed.
%
is_synchronized(subject(S)) :-
	!,
	\+ thread_peek_message(ask_queries, request(subject(S),_,_,_)).

is_synchronized(Q) :-
	\+ thread_peek_message(ask_queries, request(_,query(Q,_),_,_)).

%%
% Block as long as a query is queued or being processed in an asynch ask operation.
%
synchronize(Term) :-
	% TODO: get a notification instead of busy waiting
	% TODO: prefer requests for which synchronize waits,
	%        they could be added to some queue and handled in get_request
	repeat,
	( is_synchronized(Term)
	-> ( ! )
	;  ( sleep(0.02), fail )
	).

		 /*******************************
		 *	    TERM EXPANSION     		*
		 *******************************/

%%
% Term expansion for *ask* rules using the (?>) operator.
% The head is rewritten as ask head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((?>(Head,Goal)),Expansions) :-
  strip_module_(Head,Module,Term),
  once((ground(Module);prolog_load_context(module,Module))),
  findall(Expansion, (
    expand_predicate_(Module,Term,Expansion);
    expand_ask_query_(Term,Goal,Expansion)
  ), Expansions).

%%
% Expand `f(..) ?> ...` to `f(..) :- lang_query:ask(f(..))`
%
expand_predicate_(Module,Head,(:-(HeadExpanded,lang_query:ask(Term)))) :-
  functor(Head,Functor,Arity),
  Indicator=(:(Module,(/(Functor,Arity)))),
  \+ current_predicate(Indicator),
  length(Args,Arity),
  Term=..[Functor|Args],
  HeadExpanded=(:(Module,Term)).

%%
expand_ask_query_(Head,Goal,(:-(HeadExpanded,GoalExpanded))) :-
  expand_ask_term_(QScope->_,_->FScope,
                   Goal,GoalExpanded),
  HeadExpanded = lang_query:ask2(Head,QScope->FScope),
  !.

%%
expand_ask_term_(QS0->QSn, FS0->FSn,
        (','(First0,Rest0)),
        (','(First1,Rest1))) :-
  !,
  expand_ask_term_(QS0->QS1,FS0->FS1,First0,First1),
  expand_ask_term_(QS1->QSn,FS1->FSn,Rest0,Rest1).
% call unscoped goal
expand_ask_term_(QS->QS, FS->FS, { Goal }, Goal ).
% options
expand_ask_term_(QS->QS, FS->FS, options(X), (QS=[X,_])).
expand_ask_term_([Opt0,QS]->[Opt1,QS], FS->FS, option(X),
    ( merge_options([X],Opt0,Opt1)) ).
% scope
expand_ask_term_(QS->QS, FS->FS, query_scope(X), (QS=[_,X])).
expand_ask_term_(QS->QS, FS->FS, fact_scope(X),  (FS=X)).
expand_ask_term_([Cfg,QS]->[Cfg,Qx], FS->FS, scope(K,X),
    ( scope_update(QS,K,X,Qx) )).
expand_ask_term_([Cfg,QS]->[Cfg,Qx], FS->FS, unscope(K,X),
    ( scope_update(QS,K,X,Qx) )).
% call scoped predicate
expand_ask_term_(QS->QS, FS->Fx, call(X,Q0),
    ( lang_query:context_update_(QS,Q0,Q1),
      lang_query:ask(X,Q1->F1),
      scope_intersect(FS,F1,Fx) )).
expand_ask_term_(QS->QS, FS->Fx, call(X),
    ( lang_query:ask(X,QS->F1),
      scope_intersect(FS,F1,Fx) )).
expand_ask_term_(QS->QS, FS->Fx, Goal,
    ( lang_query:ask(Goal,QS->F1),
      scope_intersect(FS,F1,Fx) )).

%%
% Term expansion for *tell* rules using the (+>) operator.
% The head is rewritten as tell/2 head, and the goal is
% expanded such that the contextual parameter is taken into account.
%
user:term_expansion((+>(Head,Goal)),
                    (:-(lang_query:tell(Term,Scope),GoalExpanded))) :-
  %%
  strip_module_(Head,_,Term),
  expand_tell_term_(Scope->_,Goal,GoalExpanded).

%%
%
expand_tell_term_(QS0->QSn,
        (','(First0,Rest0)),
        (','(First1,Rest1))) :-
  !,
  expand_tell_term_(QS0->QS1,First0,First1),
  expand_tell_term_(QS1->QSn,Rest0,Rest1).
% call unscoped goal
expand_tell_term_(QS->QS, { Goal }, Goal ).
% options
expand_tell_term_(QS->QS, options(X), (QS=[X,_])).
expand_tell_term_([Opt0,QS]->[Opt1,QS], option(X),
    ( merge_options([X],Opt0,Opt1)) ).
% scope
expand_tell_term_(QS->QS, fact_scope(X), (QS=[_,X])).
expand_tell_term_([Opt,QS]->[Opt,Qx], scope(K,X),
    ( scope_update(QS,K,X,Qx) )).
expand_tell_term_([Opt,QS]->[Opt,Qx], unscope(K,X),
    ( scope_remove(QS,K,X,Qx) )).
% notifications
% TODO: would be nice to notify at the end
%           of a query, not instantly
expand_tell_term_(QS->QS, notify(X), notify(X)).
% call scoped predicate
expand_tell_term_(QS->QS, call(Statement,Q0),
    ( lang_query:context_update_(QS,Q0,Q1),
      lang_query:tell(Statement,Q1) )).
expand_tell_term_(QS->QS, call(Statement),
    ( lang_query:tell(Statement,QS) )).
expand_tell_term_(QS->QS, update(Statement),
    ( lang_query:update(Statement,QS) )).
expand_tell_term_(QS->QS, Statement,
    ( lang_query:tell(Statement,QS) )).

%%
% Term expansion for *tell-ask* rules using the (?+>) operator.
% These are basically clauses that can be used in both contexts.
%
% Consider for example following rule:
%
%     is_event(Entity) ?+>
%       has_type(Entity, dul:'Event').
%
% This is valid because, in this case, has_type/2 has
% clauses for ask and tell.
%
user:term_expansion((?+>(Head,Goal)), [X1,X2]) :-
  user:term_expansion((?>(Head,Goal)),X1),
  user:term_expansion((+>(Head,Goal)),X2).

%%
% NOTE: SWI strip_module acts strange
strip_module_(:(Module,Term),Module,Term) :- !.
strip_module_(Term,_,Term).

%%
context_update_([Options0,Scope0],New,[Options1,Scope1]) :-
  ( option(scope(NewScope),New) ->
    scope_update(Scope0,NewScope,Scope1) ;
    Scope1=Scope0
  ),
  ( option(options(NewOptions),New) ->
    merge_options(NewOptions,Options0,Options1) ;
    Options1=Options0
  ).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- begin_tests('lang_query').

:- use_module(library('semweb/rdf_db'),
		[ rdf_meta/1, rdf_global_term/2 ]).

:- rdf_meta(test_asynch_answer(t)).
:- rdf_meta(test_asynch_answer0(+,t)).

:- rdf_db:rdf_register_ns(dul,
		'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

% used to get answers from asynch ask
:- message_queue_create(_,[alias(ask_test_results)]).

test_asynch_finish :-
	% pop out all messages from ask_test_results queue
	repeat,
	\+ thread_get_message(ask_test_results,_,[timeout(0)]),
	!.

test_asynch_has_finished(Query) :-
	% an asynch ask test has finished when the query is processed completely,
	% end all messages were popped out the queue
	assert_true(is_synchronized(Query)),
	assert_true(message_queue_property(ask_test_results,size(0))).

test_asynch_ask(Mode,Query) :-
	% asynch ask test sets the *callback* option and hooks it
	% to *thread_send_message*
	current_scope(QScope),
	Callback=..[Mode,thread_send_message(ask_test_results)],
	ask(Query, [[callback(Callback)],QScope]).

test_asynch_answer0(Actual,Expected) :-
	assert_true(is_list(Actual)),
	assert_true(Actual=[_,_]),
	Actual=[Fact,FScope],
	assert_true(ground(Fact)),
	assert_true(Fact=Expected),
	assert_true(is_dict(FScope)).

test_asynch_answer(Expected) :-
	thread_get_message(ask_test_results,Actual),
	test_asynch_answer0(Actual,Expected).

test('ask asynch all det') :-
	rdf_global_term(triple(dul:'Object',rdf:type,_),Query),
	%%
	test_asynch_ask(all,Query),
	thread_get_message(ask_test_results,Answers),
	assert_true(is_list(Answers)),
	assert_true(length(Answers,1)),
	Answers=[Answer],
	test_asynch_answer0(Answer,
		triple(dul:'Object',rdf:type,owl:'Class')),
	test_asynch_has_finished(Query).

test('ask asynch one det') :-
	rdf_global_term(triple(dul:'Object',rdf:type,_),Query),
	%%
	test_asynch_ask(one,Query),
	test_asynch_answer(triple(dul:'Object',rdf:type,owl:'Class')),
	test_asynch_has_finished(Query).

test('ask asynch each det') :-
	rdf_global_term(triple(dul:'Object',rdf:type,_),Query),
	%%
	test_asynch_ask(each,Query),
	test_asynch_answer(triple(dul:'Object',rdf:type,owl:'Class')),
	test_asynch_has_finished(Query).

test('ask asynch all nondet') :-
	rdf_global_term(triple(_,rdfs:subClassOf,dul:'Object'),Query),
	%%
	test_asynch_ask(all,Query),
	thread_get_message(ask_test_results,Answers),
	assert_true(is_list(Answers)),
	assert_true(Answers=[_|_]),
	findall(X, member([triple(X,_,_),_],Answers), SubClasses),
	assert_true(SubClasses=[_|_]),
	assert_true(member(dul:'PhysicalObject',SubClasses)),
	test_asynch_has_finished(Query).

test('ask asynch one nondet') :-
	rdf_global_term(triple(dul:'Object',rdfs:subClassOf,_),Query),
	%%
	test_asynch_ask(one,Query),
	test_asynch_answer(Query),
	test_asynch_has_finished(Query).

test('ask asynch each nondet') :-
	rdf_global_term(triple(dul:'Object',rdfs:subClassOf,_),Query),
	%%
	test_asynch_ask(each,Query),
	test_asynch_answer(Query),
	test_asynch_answer(Query),
	assert_true(synchronize(Query)),
	assert_true(is_synchronized(Query)),
	test_asynch_finish.

test('ask asynch multiple queries') :-
	rdf_global_term(triple(dul:'Object',rdfs:subClassOf,_),Query),
	QueryCount is 20,
	% create QueryCount ask requests
	forall(
		between(1,QueryCount,_),
		test_asynch_ask(all,Query)
	),
	% poll QueryCount results from message queue
	forall(
		between(1,QueryCount,_),
		thread_get_message(ask_test_results,_)
	),
	test_asynch_has_finished(Query).

:- end_tests('lang_query').
