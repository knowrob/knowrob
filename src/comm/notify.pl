:- module(notify,
    [ notify(t)
    ]).
/** <module> Broadcasting of notifications.

@author Daniel Beßler
@license BSD
*/

:- dynamic notify_hook/1.
:- multifile notify_hook/1.

%% notify(+Term) is nondet.
%
% Broadcast a notification term.
% Listeners register by declaring a clause of notify:notify_hook/1.
%
% @param Term A notification term.
%
notify(Term) :-
  ( notify_hook(Term) *-> true ; true ).
  
