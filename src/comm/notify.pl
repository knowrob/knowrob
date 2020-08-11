:- module(notify,
    [ notify(t),
      notify_synchronize(t)
    ]).
/** <module> Broadcasting of notifications.

@author Daniel Beßler
@license BSD
*/

:- dynamic notify_hook/1.
:- dynamic notify_thread/1.
:- multifile notify_hook/1.

%%
% A worker thread processing notify messages.
%
notify_loop :-
	repeat,
	thread_get_message(Msg),
	catch(
		forall(notify_hook(Msg),true),
		Exception,
		print_message(error,notify_exception(Exception))
	),
	% fallback to 'repeat' above
	fail.

% create threads when this module is consulted
:- thread_create(notify_loop,Thread),
   assertz(notify_thread(Thread)).

%% notify_synch(+Term) is nondet.
%
notify_synchronize(Term) :-
	notify_thread(Thread),
	repeat,
	(	thread_peek_message(Thread,Term)
	->	( sleep(0.01), fail )
	;	true
	),
	!.

%% notify(+Term) is nondet.
%
% Broadcast a notification term.
% Listeners register by declaring a clause of notify:notify_hook/1.
%
% @param Term A notification term.
%
notify(individual(X)) :-
	!,
	(	is_object(X) -> notify(object(X))
	;	is_event(X)  -> notify(event(X))
	;	true
	).

notify(Term) :-
	notify_thread(Thread),
	thread_send_message(Thread, Term).
