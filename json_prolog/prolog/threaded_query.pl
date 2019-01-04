
:- module(threaded_query,
    [
      queryt_create/2,
      queryt_finish/1,
      queryt_has_next/1,
      queryt_next_solution/2,
      queryt_one_solution/2
    ]).

:- use_module(library(thread_pool)).

:- thread_pool_create(queryt_pool, 6, [backlog(infinite)]).

:- dynamic queryt_term/2.
:- dynamic queryt_call_active/1.

random_id(Id) :-
  randseq(8,25,Seq_random),
  maplist(plus(65),Seq_random,Alpha_random),
  atom_codes(Id,Alpha_random).

queryt_unique_id(QueryId) :-
  random_id(RandomId),
  ( queryt_term(RandomId,_) ->
    queryt_unique_id(QueryId) ;
    QueryId=RandomId ).

queryt(QueryId,query(QueryId,MsgQueue)) :-
  queryt_term(QueryId,MsgQueue).

%% queryt_create(+GoalAtom, -QueryId)
%
% Assigns Goal to a worker thread in which it is called.
% The worker thread processing the query can be
% referred to with QueryId.
%
queryt_create(GoalAtom, QueryId) :-
  queryt_unique_id(QueryId),
  %
  message_queue_create(MsgQueue),
  assertz(queryt_term(QueryId,MsgQueue)),
  %
  thread_create(
    queryt_run(query(QueryId,MsgQueue),GoalAtom), _,
    [alias(QueryId)]
  ).

queryt_run(QueryCtx, GoalAtom) :-
  queryt_call_begin(QueryCtx),
  ignore(forall(
    queryt_run_call_and_send(QueryCtx,GoalAtom),
  (
    queryt_call_end(QueryCtx),
    % Sleep until next solution requested
    queryt_wait_request(QueryCtx),
    queryt_call_begin(QueryCtx)
  ))),
  once((
    queryt_finished_internal(QueryCtx);
    queryt_call_end(QueryCtx))).
queryt_run_call_and_send(query(_,MsgQueue), GoalAtom) :-
  % Unify Args with a list of variables,
  % each sharing with a variable that appears only once in Goal
  read_term_from_atom(GoalAtom,Goal,[variable_names(Args)]),
  catch(call(Goal), E, (
    print_message(error, E),
    fail
  )),
  % Place solution(..) term in MsgQueue
  findall([Name,Value], member(Name=Value,Args), ArgsList),
  thread_send_message(MsgQueue, solution(ArgsList)).

queryt_call_begin(query(QueryId,MsgQueue)) :-
  \+ queryt_finished(QueryId),
  % remove any remaining `call_finished` msgs from queue
  ignore(thread_get_message(MsgQueue, call_finished, [timeout(0)])),
  assertz(queryt_call_active(QueryId)).
queryt_call_end(query(QueryId,MsgQueue)) :-
  \+ queryt_finished(QueryId),
  retract(queryt_call_active(QueryId)),
  % wake up any thread waiting on this call to finish
  thread_send_message(MsgQueue, call_finished).

queryt_wait_call(QueryCtx) :-
  % the query has finished already
  queryt_finished_internal(QueryCtx), !.
queryt_wait_call(query(QueryId,_)) :-
  % call is not active
  \+ queryt_call_active(QueryId), !.
queryt_wait_call(query(_,MsgQueue)) :-
  % wait until the call finished
  thread_get_message(MsgQueue, call_finished).

queryt_wait_request(query(QueryId,MsgQueue)) :-
  \+ queryt_finished(QueryId),
  thread_get_message(MsgQueue, request).

queryt_acquire(QueryId,QueryCtx) :-
  queryt(QueryId, QueryCtx),
  % wait for active `call`
  queryt_wait_call(QueryCtx),
  % make sure query has not finished during wait
  \+ queryt_finished(QueryId).

%% queryt_has_next(+QueryId)
%
queryt_has_next(QueryId) :-
  queryt_acquire(QueryId,query(_,MsgQueue)),
  thread_peek_message(MsgQueue, solution(_)),!.

%% queryt_one_solution(+QueryId,-Solution)
%
queryt_one_solution(QueryId,Solution) :-
  queryt_next_solution(QueryId,Solution),
  queryt_finish(QueryId).

%% queryt_next_solution(+QueryId,-Next)
%
queryt_next_solution(QueryId,Next) :-
  queryt_acquire(QueryId,query(_,MsgQueue)),
  thread_get_message(MsgQueue, solution(Next), [timeout(0)]),
  % wake up the worker thread for finding more solutions
  thread_send_message(MsgQueue, request),!.

queryt_finished(QueryId) :-
  \+ queryt(QueryId, _), !.
queryt_finished(QueryId) :-
  queryt(QueryId,query(_,MsgQueue)),
  thread_peek_message(MsgQueue, finished).
queryt_finished_internal(query(QueryId,_)) :-
  \+ queryt(QueryId, _), !.
queryt_finished_internal(query(_,MsgQueue)) :-
  thread_peek_message(MsgQueue, finished).

%% queryt_finish(+QueryId)
queryt_finish(QueryId) :-
  queryt_finished(QueryId), !.
queryt_finish(QueryId) :-
  queryt(QueryId,query(_,MsgQueue)),
  retract(queryt_term(QueryId,MsgQueue)),
  retractall(queryt_call_active(QueryId)),
  % wake up everyone
  once((thread_peek_message(MsgQueue, finished);
        thread_send_message(MsgQueue, finished))),
  thread_send_message(MsgQueue, request),
  thread_send_message(MsgQueue, call_finished),
  %
  message_queue_destroy(MsgQueue).
