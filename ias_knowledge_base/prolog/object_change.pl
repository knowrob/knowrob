
:- module(object_change,
    [
      transformed_into/2,
      action_effects/2,
      transformed_into_transitive/2,
      comp_thermicallyConnectedTo/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- use_module(library('action_effects')).
:- use_module(library('processes')).


:- owl_parser:owl_parse('../owl/object-change.owl', false, false, true).
:- owl_parser:owl_parse('../owl/pancake-making.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://ias.cs.tum.edu/kb/object-change.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake, 'http://ias.cs.tum.edu/kb/pancake-making.owl#', [keep(true)]).

:-  rdf_meta
    transformed_into(r, r),
    transformed_into_transitive(r, r),
    comp_thermicallyConnectedTo(r,r),
    action_effects(r,r).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% action effects
%

action_effects(Action, PostActors) :-

  % project what is happening when performing the action
  project_action(Action),

  % check for processes that got triggered
  process,

  rdf_has(Action, knowrob:postActors, PostActors).

% definitions of action effects in module action_effects



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% processes
%

% definitions of processes in module processes


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% object transformations
%

transformed_into(From, To) :-
  ( owl_has(Event, knowrob:thingIncorporated, From);
    owl_has(Event, knowrob:objectAddedTo, From);
    owl_has(Event, knowrob:inputsDestroyed, From);
    owl_has(Event, knowrob:inputsCommitted, From);
    owl_has(Event, knowrob:transformedObject, From);
    owl_has(Event, knowrob:objectRemoved, From);
    owl_has(Event, knowrob:objectOfStateChange, From);
    owl_has(Event, knowrob:outputsRemaining, From) ),

  ( owl_has(Event, knowrob:outputsRemaining, To);
    owl_has(Event, knowrob:outputsCreated, To)).



% transitivity
transformed_into_transitive(From, To) :-
  transformed_into(From, To).

transformed_into_transitive(From, To) :-
  transformed_into(From, Sth),
  From\=Sth,
  transformed_into_transitive(Sth, To).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% process-relevant object relations
%

% a heat path exists between two objects if they are either
% on top of each other or if one contains the other one
comp_thermicallyConnectedTo(Obj1, Obj2) :-
  rdf_triple(knowrob:'on-Physical', Obj1, Obj2);
  rdf_triple(knowrob:'on-Physical', Obj2, Obj1).


comp_thermicallyConnectedTo(Obj1, Obj2) :-
  rdf_triple(knowrob:'in-ContGeneric', Obj1, Obj2);
  rdf_triple(knowrob:'in-ContGeneric', Obj2, Obj1).



