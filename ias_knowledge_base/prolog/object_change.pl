
:- module(object_change,
    [
      transformed_into/2,
      action_effect/2,
      transformed_into_transitive/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).

:- owl_parser:owl_parse('../owl/object-change.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://ias.cs.tum.edu/kb/object-change.owl#', [keep(true)]).

:-  rdf_meta
    transformed_into(r, r),
    transformed_into_transitive(r, r),
    action_effect(r,r).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% action effects
%


% Cracking an egg
action_effect(Action, Event) :-

  owl_individual_of(Action, knowrob:'Cracking'),
  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_individual_of(Obj, knowrob:'Egg-Chickens'),!,

  rdf_instance_from_class(object_change:'PhysicalTransformationEvent', Event),
  rdf_assert(Event, object_change:inputsDestroyed, Obj),

  rdf_instance_from_class(knowrob:'EggShell', Shell),
  rdf_assert(Event, object_change:outputsCreated, Shell),

  rdf_instance_from_class(object_change:'EggYolk-Food', Yolk),
  rdf_assert(Event, object_change:outputsCreated, Yolk).


% Cutting off a slice of bread
action_effect(Action, Event) :-

  owl_individual_of(Action, object_change:'CuttingAPieceOfFood'),
  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_individual_of(Obj, knowrob:'Bread'),!,

  rdf_instance_from_class(object_change:'CuttingOffAPiece', Event),
  rdf_assert(Event, object_change:objectOfStateChange, Obj),

  rdf_instance_from_class(object_change:'BreadSlice', Slice),
  rdf_assert(Event, object_change:objectRemoved, Slice).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% object transformations
%

transformed_into(From, To) :-
  owl_individual_of(Event, object_change:'CuttingOffAPiece'),
  owl_has(Event, object_change:objectOfStateChange, From),
  owl_has(Event, object_change:objectRemoved, To).


transformed_into(From, From) :-
  owl_individual_of(Event, object_change:'CuttingOffAPiece'),
  owl_has(Event, object_change:objectOfStateChange, From).

transformed_into(From, To) :-
  owl_individual_of(Event, object_change:'CreationOrDestructionEvent'),
  owl_has(Event, object_change:inputsDestroyed, From),
  owl_has(Event, object_change:outputsCreated, To).


transformed_into(From, To) :-
  owl_individual_of(Event, knowrob:'Incorporation-PhysicalAction'),
  owl_has(Event, object_change:thingIncorporated, From),
  owl_has(Event, object_change:objectAddedTo, To).

transformed_into(From, From) :-
  owl_individual_of(Event, knowrob:'Incorporation-PhysicalAction'),
  owl_has(Event, object_change:objectAddedTo, From),!.



% transitivity
transformed_into_transitive(From, To) :-
  transformed_into(From, To).

transformed_into_transitive(From, To) :-
  transformed_into(From, Sth),
  From\=Sth,
  transformed_into_transitive(Sth, To).






