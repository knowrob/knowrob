:- module(obda,
    [ obda_add/1,
      obda_remove/1,
      obda_import/1,
      obda_export/1,
      obda_whipe/0,
      access(t,t)
    ]).
/** <module> Predicates for Ontology-based Data Access (OBDA).

@author Daniel Beßler
@license BSD
*/

:- dynamic obda_client_/2.

%% obda_add(+Module) is det.
%
% Registers a Prolog Module that implements the OBDA
% interface.
%
% @param Module a Prolog module.
%
obda_add(Module) :-
  forall(
    call((:(Module,can_access(X)))),
    assertz(obda_client_(Module,X))).

%% obda_remove(+Module) is det.
%
% Removes a previously registered OBDA module.
%
% @param Module a Prolog module.
%
obda_remove(Module) :-
  retractall(obda_client_(Module,_)).

%% obda_import(+Directory) is det.
%
% Calls import on any registered OBDA module
% with given directory.
%
% @param Directory a Prolog module.
%
obda_import(Directory) :-
  forall(
    obda_client_(Module,_),
    call( (:(Module,import(Directory))) )).

%% obda_export(+Directory) is det.
%
% Calls export on any registered OBDA module
% with given directory.
%
% @param Directory a Prolog module.
%
obda_export(Directory) :-
  forall(
    obda_client_(Module,_),
    call( (:(Module,export(Directory))) )).

%% obda_whipe is det.
%
% Whipe all OBDA databases of registered modules.
%
obda_whipe :-
  forall(
    obda_client_(Module,_),
    call( (:(Module,whipe)) )).

%% access(+DataStatement,+Scope) is nondet.
%
% Try accessing a data statement with registered
% OBDA modules by calling their access predicate.
%
% @param DataStatement a term holds(Subject,Property,Value).
% @param Scope a term of the form QueryScope->FactScope.
%
access(holds(Subject,Property,Value), QScope->FScope) :-
  % find OBDA client for Property
  obda_client_(Module,Property),
  % call it
  call( (:(Module,access(
    Subject,Property,Value,QScope,FScope))) ).
