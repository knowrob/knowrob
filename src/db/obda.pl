:- module(obda,
    [ obda_add/1,
      obda_remove/1,
      obda_import/1,
      obda_export/1,
      obda_whipe/0,
      access(t,t)
    ]).
/** <module> ....

@author Daniel Beßler
@license BSD
*/

:- dynamic obda_client_/2.

%%
%
%
obda_add(Module) :-
  forall(
    call((:(Module,can_access(Property)))),
    assertz(obda_client_(Module,X))
  ).

%%
%
%
obda_remove(Module) :-
  retractall(obda_client_(Module,_)).

%%
%
%
obda_import(Directory) :-
  forall(
    obda_client_(Module,_),
    call( (:(Module,import(Directory))) ),
  ).

%%
%
%
obda_export(Directory) :-
  forall(
    obda_client_(Module,_),
    call( (:(Module,export(Directory))) ),
  ).

%%
%
%
obda_whipe :-
  forall(
    obda_client_(Module,_),
    call( (:(Module,whipe(Directory))) ),
  ).

%%
%
%
access(holds(Subject,Property,Value), QScope->FScope) :-
  % find OBDA client for Property
  obda_client_(Module,Property),
  % call it
  call( (:(Module,access(
    Subject,Property,Value,QScope,FScope))) ).
