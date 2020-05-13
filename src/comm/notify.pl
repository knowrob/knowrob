:- module(notify,
    [ notify(t)
    ]).

:- dynamic notify_hook/1.
:- multifile notify_hook/1.

%user:term_expansion(
  %(:-(:(_Module, notify_hook(Term),Goal))),
  %(:-(:(notify,  notify_hook(Term),Goal)))).

notify(Term) :-
  ( notify_hook(Term) *-> true ; true ).
  
