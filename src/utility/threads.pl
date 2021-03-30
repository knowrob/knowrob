:- module(thread_utils,
    [ message_queue_materialize/2,  % +Queue, -Term
      worker_pool_create/1,         % +PoolID
      worker_pool_create/2,         % +PoolID, +Options
      worker_pool_start_work/3,     % +PoolID, +WorkID, +Goal
      worker_pool_stop_work/2,      % +PoolID, +WorkID
      worker_pool_join/2            % +PoolID, +WorkID
    ]).
/** <module> Threading utilities.

@author Daniel Beßler
@license BSD
*/

:- dynamic worker_pool/4.
:- dynamic num_workers/2.
:- dynamic pending_join/3.

%% message_queue_materialize(+Queue, -Term) is nondet.
%
% Retrieves and removes Term from queue.
% This will block until at least one message is available in the queue
% before returning a result or failing.
% A choicepoint is created for the next queue element until end_of_stream reached.
% It is not ensured that this call is deterministic in case
% there is only one message before end_of_stream
% (it is deterministic if end_of_stream was already queued
% when the last message was retrieved).
%
% @param Queue a message queue.
% @param Term queued message
% @see https://www.swi-prolog.org/pldoc/man?section=msgqueue
%
message_queue_materialize(Queue, Term) :-
	thread_get_message(Queue, This),
	message_queue_materialize(Queue, This, Term).

%
message_queue_materialize(_, end_of_stream, _) :- !, fail.
message_queue_materialize(_, end_of_stream(Term), Term) :- !.
message_queue_materialize(_, error(Error), _) :- !, throw(Error).
message_queue_materialize(Queue, This, Term) :-
	(	thread_get_message(Queue, Next, [timeout(0)])
	% there are multiple messages queued
	->	(	Next==end_of_stream
		->	Term=This
		;	Term=This
		;	message_queue_materialize(Queue, Next, Term)
		)
	% there was only one message queued
	;	Term=This
	;	message_queue_materialize(Queue, Term)
	).

% message queue for work goals
pool_work_queue(WorkerPool, WorkQueue) :-
	worker_pool(WorkerPool, WorkQueue, _, _).

% message queue for currently active goals
pool_active_queue(WorkerPool, ActiveQueue) :-
	worker_pool(WorkerPool, _, ActiveQueue, _).

% options stored for a pool
pool_option(WorkerPool, Option) :-
	worker_pool(WorkerPool, _, _, OptionList),
	option(Option, OptionList).

pool_option(WorkerPool, Option, Default) :-
	worker_pool(WorkerPool, _, _, OptionList),
	option(Option, OptionList, Default).
	

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
% Creates a new thread pool with worker threads. Options include:
%
%     - initial_pool_size(InitialSize)
%     Determines the number of threads initially started.  Default is 2.
%
% @param WorkerPool the worker pool name.
% @param Options additional options.
%
worker_pool_create(WorkerPool, Options) :-
	option(initial_pool_size(InitialSize), Options, 2),
	% create message queues used by the pool
	message_queue_create(WorkQueue),
	message_queue_create(ActiveQueue),
	% assert some dynamic predicates
	asserta(num_workers(WorkerPool, 0)),
	assertz(worker_pool(WorkerPool, WorkQueue, ActiveQueue, Options)),
	% finall create worker threads
	worker_create(WorkerPool, InitialSize).

% create Count new worker threads
worker_create(WorkerPool, Count) :-
	% create threads
	forall(
		between(1,Count,_),
		thread_create(worker_thread(WorkerPool), _, [debug(false)])
	),
	% update counter
	num_workers(WorkerPool, OldNumWorkers),
	NewNumWorkers is OldNumWorkers + Count,
	retractall(num_workers(WorkerPool,_)),
	asserta(num_workers(WorkerPool,NewNumWorkers)).

% resize worker pool if there is more work then workers
pool_grow(WorkerPool) :-
	pool_option(WorkerPool, max_size(MaxSize), -1),
	pool_active_queue(WorkerPool, ActiveQueue),
	with_mutex(WorkerPool, (
		num_workers(WorkerPool, CurrentNumWorkers),
		message_queue_property(ActiveQueue, size(NeededWorkers)),
		% bounded growing
		(	MaxSize > 0
		->	NeededWorkers0 is min(MaxSize,NeededWorkers)
		;	NeededWorkers0 = NeededWorkers
		),
		Diff is NeededWorkers0 - CurrentNumWorkers,
		(	Diff =< 0 -> true
		;	worker_create(WorkerPool, Diff)
		)
	)).


%% worker_pool_start_work(+WorkerPool, -WorkID, +WorkerGoal) is det.
%
% Schedules a new work goal.
% worker_pool_stop_work/2 must be called once the work is done.
% This is usually done by wrapping it into a call of setup_call_cleanup/3.
%
% @param WorkerPool the worker pool name.
% @param WorkID the work ID.
% @param WorkerGoal the goal of each worker.
%
worker_pool_start_work(WorkerPool, WorkID, WorkerGoal) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	pool_work_queue(WorkerPool, WorkQueue),
	% add work ID to active queue
	thread_send_message(ActiveQueue, work(WorkID)),
	% add a new work goal to the queue
	thread_send_message(WorkQueue, work(WorkID, WorkerGoal)),
	% increase number of threads if needed
	pool_grow(WorkerPool).

%% worker_pool_join(+PoolID, +WorkID) is det.
%
% Block the current thread until work is done.
%
worker_pool_join(PoolID, WorkID) :-
	pool_active_queue(PoolID, ActiveQueue),
	\+ thread_peek_message(ActiveQueue, work(WorkID)),
	!.

worker_pool_join(PoolID, WorkID) :-
	thread_self(ThisThread),
	% mutex is locked as long as work is active, so entering mutex
	% is only possible when no work is active.
	with_mutex(PoolID, (
		assertz(pending_join(PoolID, WorkID, ThisThread))
	)),
	thread_get_message(wake_up),
	with_mutex(PoolID, (
		retract(pending_join(PoolID, WorkID, ThisThread))
	)).

%
pool_wake_up(PoolID, WorkID) :-
	with_mutex(PoolID, (
		forall(
			pending_join(PoolID, WorkID, WaitingThread),
			thread_send_message(WaitingThread, wake_up)
		)
	)).


%% worker_pool_stop_work(+WorkerPool, -WorkID) is det.
%
% Unschedule any remaining work associated to WorkID.
% This may not immediately shut-down all operations,
% but will prevent any more results to be communicated,
% and new operations being started.
%
% @param WorkerPool the worker pool name.
% @param WorkID the work ID.
%
worker_pool_stop_work(WorkerPool, WorkID) :-
	% TODO: force threads to exit
	worker_pool_stop_work1(WorkerPool, WorkID, _Count).

%
worker_pool_stop_work1(WorkerPool, WorkID, Count) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_get_message(ActiveQueue, work(WorkID), [timeout(0)]),
	!,
	worker_pool_stop_work1(WorkerPool, WorkID, Count0),
	Count is Count0 + 1.
worker_pool_stop_work1(_, _, 0) :- !.

% a goal executed for each worker thread.
% each worker listens to the same WorkQueue
% with messages encoding a callable worker goal
worker_thread(WorkerPool) :-
	pool_work_queue(WorkerPool, WorkQueue),
	pool_active_queue(WorkerPool, ActiveQueue),
	repeat,
	thread_get_message(WorkQueue, Msg),
	Msg=work(WorkID, WorkerGoal),
	% TODO: use catch_with_backtrace
	catch(
		forall(worker_thread(work(WorkID, WorkerGoal), WorkerPool), true),
		Error,
		worker_thread_error(Error)
	),
	ignore(
		thread_get_message(ActiveQueue, work(WorkID), [timeout(0)])
	),
	% send notification that work is over if queue is now empty
	(	thread_peek_message(ActiveQueue, work(WorkID)) -> true
	;	pool_wake_up(WorkerPool, WorkID)
	),
	% jump back to repeat/0
	fail.

worker_thread(work(WorkID, WorkerGoal), WorkerPool) :-
	work_is_ongoing(WorkerPool, WorkID),
	call(WorkerGoal),
	(	work_is_ongoing(WorkerPool, WorkID) -> true
	;	(!, fail)
	).

%
worker_thread_error('$aborted') :- !.
worker_thread_error(error(existence_error(message_queue,_),_)) :- !.
worker_thread_error(Error) :- log_warning(worker_thread(Error)).

% true if there is a reference to WorkID in ActiveQueue
work_is_ongoing(WorkerPool, WorkID) :-
	pool_active_queue(WorkerPool, ActiveQueue),
	thread_peek_message(ActiveQueue, work(WorkID)).
