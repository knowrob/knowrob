:- module(worker_pool,
    [ worker_pool_create/1,
      worker_pool_create/2,
      worker_pool_call/3,
      worker_pool_start_work/4,
      worker_pool_stop_work/2,
      worker_pool_pop/3,
      worker_pool_materialize/3
    ]).
/** <module> A worker pool implementation.

@author Daniel Beßler
@license BSD
*/

:- dynamic worker_pool/3.

% message queue for work goals
pool_work_queue(WorkerPool, WorkQueue) :-
	worker_pool(WorkerPool, WorkQueue, _).

% message queue for currently active goals
pool_active_queue(WorkerPool, ActiveQueue) :-
	worker_pool(WorkerPool, _, ActiveQueue).


%% worker_pool_create(+WorkerPool) is det.
%
% Same as worker_pool_create/2 with empty options.
%
% @param WorkerPool the worker pool name.
%
worker_pool_create(WorkerPool) :-
	worker_pool_create(WorkerPool, []).


%% worker_pool_create(+WorkerPool, +Options) is det.
%
% Creates a new thread pool with worker threads.
%
% @param WorkerPool the worker pool name.
% @param Options additional options.
%
worker_pool_create(WorkerPool, Options) :-
	% TODO: better fallback
	% TODO: thread and message queue options
	option(size(Size), Options, 5),
	message_queue_create(WorkQueue),
	message_queue_create(ActiveQueue),
	assertz(worker_pool(WorkerPool, WorkQueue, ActiveQueue)),
	forall(
		between(1,Size,_),
		thread_create(worker_thread(WorkerPool), _, [debug(false)])
	).


%% worker_pool_call(+WorkerPool, +WorkerGoal, +GeneratorGoal) is nondet.
%
% Call WorkerGoal for each instantiation inferred by calling GeneratorGoal.
% GeneratorGoal is called in a thread too, and can be terminated early
% e.g. in case the call is wrapped in once/1.
%
% @param WorkerPool the worker pool name.
% @param WorkerGoal the goal of each worker.
% @param GeneratorGoal the input generator goal.
%
worker_pool_call(WorkerPool, WorkerGoal, GeneratorGoal) :-
	setup_call_cleanup(
		% create a message queue for work results
		worker_pool_start_work(WorkerPool, WorkerGoal, GeneratorGoal, WorkID),
		% do some threaded work
		worker_pool_materialize(WorkerPool, WorkID, WorkerGoal),
		% suspend any pending computations associated to WorkID
		worker_pool_stop_work(WorkerPool, WorkID)
	).


%% worker_pool_start_work(+WorkerPool, +WorkerGoal, +GeneratorGoal, -WorkID) is det.
%
% Schedules a new work goal.
% worker_pool_stop_work/2 must be called ony the work is done.
% This is usually done by wrapping it into a call of setup_call_cleanup/3.
%
% @param WorkerPool the worker pool name.
% @param WorkerGoal the goal of each worker.
% @param GeneratorGoal the input generator goal.
% @param WorkID the work ID.
%
worker_pool_start_work(WorkerPool, WorkerGoal, GeneratorGoal, WorkID) :-
	% add work ID to active queue
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_send_message(ActiveQueue, work(WorkID)),
	% create a message queue for work results
	message_queue_create(WorkID),
	pool_push(WorkerPool, WorkID, WorkerGoal, GeneratorGoal).


%% worker_pool_stop_work(+WorkerPool, -WorkID) is det.
%
% Unschedule any remaining work associated to WorkID.
%
% @param WorkerPool the worker pool name.
% @param WorkID the work ID.
%
worker_pool_stop_work(WorkerPool, WorkID) :-
	% remove work ID from active queue
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_get_message(ActiveQueue, work(WorkID)),
	pool_remove_all_references(WorkerPool, WorkID),
	% destroy the work output queue
	message_queue_destroy(WorkID).

% each worker thread holds a reference to the work ID
pool_add_reference(WorkerPool, WorkID) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_send_message(ActiveQueue, reference(WorkID)).

% each worker thread holds a reference to the work ID
pool_remove_reference(WorkerPool, WorkID) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	ignore(thread_get_message(ActiveQueue, reference(WorkID), [timeout(0)])),
	(	thread_peek_message(ActiveQueue, reference(WorkID)) -> true
	;	thread_send_message(WorkID, result(no_solution))
	).

pool_remove_all_references(WorkerPool, WorkID) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	\+ thread_peek_message(ActiveQueue, reference(WorkID)),
	!.

pool_remove_all_references(WorkerPool, WorkID) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_get_message(ActiveQueue, reference(WorkID), [timeout(0)]),
	pool_remove_all_references(WorkerPool, WorkID).	

% true if there is any remaining reference to WorkID
work_is_ongoing(WorkerPool, WorkID) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_peek_message(ActiveQueue, work(WorkID)).

% add a new work goal to the queue
pool_push(WorkerPool, WorkID, WorkerGoal, GeneratorGoal) :-
	pool_push1(WorkerPool, WorkID, work(WorkID, WorkerGoal, GeneratorGoal)).

pool_push(WorkerPool, WorkID, WorkerGoal) :-
	pool_push1(WorkerPool, WorkID, work(WorkID, WorkerGoal)).

pool_push1(WorkerPool, WorkID, WorkerMsg) :-
	pool_work_queue(WorkerPool, WorkQueue),
	pool_add_reference(WorkerPool, WorkID),
	thread_send_message(WorkQueue, WorkerMsg).


%% worker_pool_pop(+WorkerPool, +WorkID, -Result) is semidet.
%
% Pop next result associated to WorkID from worker pool.
%
% @param WorkerPool the worker pool name.
% @param WorkID the work ID.
% @param Result instantiation of the work goal.
%
worker_pool_pop(_WorkerPool, WorkID, WorkGoal) :-
	thread_get_message(WorkID, result(Result)),
	Result \== no_solution,
	WorkGoal = Result.

%% worker_pool_materialize(+WorkerPool, +WorkID, -Result) is nondet.
%
% Pop results associated to WorkID from worker pool.
%
% @param WorkerPool the worker pool name.
% @param WorkID the work ID.
% @param Result instantiation of the work goal.
%
worker_pool_materialize(WorkerPool, WorkID, WorkGoal) :-
	thread_get_message(WorkID, result(Result)),
	worker_pool_materialize1(Result, WorkerPool, WorkID, WorkGoal).

worker_pool_materialize1(no_solution, _, _, _)   :- !, fail.
worker_pool_materialize1(Result, _, _, WorkGoal) :- WorkGoal=Result.
worker_pool_materialize1(_, WorkerPool, WorkID, WorkGoal) :-
	worker_pool_materialize(WorkerPool, WorkID, WorkGoal).

%%
work_msg_id(work(WorkID,_),WorkID).
work_msg_id(work(WorkID,_,_),WorkID).

%%
worker_thread(WorkerPool) :-
	pool_work_queue(WorkerPool, WorkQueue),
	repeat,
	% NOTE: argument _must_ be an unbound variable to make it
	%       work to share the same input queue in different worker threads
	thread_get_message(WorkQueue, WorkMsg),
	work_msg_id(WorkMsg, WorkID),
	catch(
		forall(worker_thread1(WorkMsg, WorkerPool), true),
		Error,
		print_message(warning, worker_pool_error(Error))
	),
	pool_remove_reference(WorkerPool, WorkID),
	% jump back to repeat/0
	fail.

worker_thread1(work(WorkID, WorkerGoal, GeneratorGoal), WorkerPool) :-
	work_is_ongoing(WorkerPool, WorkID),
	call(GeneratorGoal),
	(	work_is_ongoing(WorkerPool, WorkID)
	->	pool_push(WorkerPool, WorkID, WorkerGoal)
	;	(!,fail)
	).

worker_thread1(work(WorkID, WorkerGoal), WorkerPool) :-
	work_is_ongoing(WorkerPool, WorkID),
	call(WorkerGoal),
	(	work_is_ongoing(WorkerPool, WorkID)
	->	thread_send_message(WorkID, result(WorkerGoal))
	;	(!,fail)
	).

