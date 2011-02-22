
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

:- owl_parser:owl_parse('../owl/object-change.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://ias.cs.tum.edu/kb/object-change.owl#', [keep(true)]).

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

  % only project when being called for the
  % first time on this action instance
  % \+ owl_has(Action, knowrob:postActors, _),

  project_action(Action, PostActors),

  %check for processes that got triggered
  process.



% Cutting off a piece
project_action(Action, PostActors) :-

  owl_individual_of(Action, knowrob:'CuttingOffAPiece'),
  owl_has(Action, knowrob:objectActedOn, Obj),

  % only project when being called for the
  % first time on this action instance
  \+ owl_has(Action, knowrob:outputsCreated, _),

  rdf_has(Obj, rdf:type, ObjType),
  ObjType \= 'http://www.w3.org/2002/07/owl#NamedIndividual',!,

  % new objects
  rdf_instance_from_class(ObjType, Slice),

  % new relations
  rdf_assert(Action, knowrob:outputsRemaining, Obj),
  rdf_assert(Action, knowrob:outputsCreated, Slice),

  print(Obj),print(' -> '), print(Slice), print('\n'),

  rdf_has(Action, knowrob:postActors, PostActors).


% Cracking an egg
project_action(Action, PostActors) :-

  owl_individual_of(Action, knowrob:'Cracking'),
  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_individual_of(Obj, knowrob:'Egg-Chickens'),!,

  % only project when being called for the
  % first time on this action instance
  \+ owl_has(Action, knowrob:outputsCreated, _),

  % new objects
  rdf_instance_from_class(knowrob:'EggShell', Shell),
  rdf_instance_from_class(knowrob:'EggYolk-Food', Yolk),

  % new relations
  rdf_assert(Action, knowrob:inputsDestroyed, Obj),
  rdf_assert(Action, knowrob:outputsCreated, Shell),
  rdf_assert(Action, knowrob:outputsCreated, Yolk),

  print(Obj),print(' -> '), print(Shell), print('\n'),
  print(Obj),print(' -> '), print(Yolk), print('\n'),

  rdf_has(Action, knowrob:postActors, PostActors).




% Putting something onto
project_action(Action, PostActors) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingOnto'),
  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_has(Action, knowrob:toLocation, To),

  % only project when being called for the
  % first time on this action instance
  \+ owl_has(Action, knowrob:'on-Physical', _),

  rdf_has(Obj, rdf:type, ObjType),
  ObjType \= 'http://www.w3.org/2002/07/owl#NamedIndividual',!,

  % new relations
  rdf_assert(Obj, knowrob:'on-Physical', To),

  print(Obj),print(' on top of '), print(To), print('\n'),

  rdf_has(Action, knowrob:postActors, PostActors).





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% processes
%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% baking process
process :-


  % % % % % % % % % % % % %
  % Individuals changed in the process
  owl_individual_of(Dough, knowrob:'Dough'),


  % % % % % % % % % % % % %
  % Preconditions (outside of QP)
  rdf_triple(knowrob:'thermicallyConnectedTo', Dough, HeatSource),


  % % % % % % % % % % % % %
  % QuantityConditions (prerequisites for the process to be active, inside of QP)

        % read temperature of the dough; default to 20 deg
        ( (rdf_triple(knowrob:temperatureOfObject, Dough, TempDough),
          strip_literal_type(TempDough, TDatom),
          term_to_atom(TDterm, TDatom)) ;
          TDterm=20 ),

        % read temperature of the heat source object; default to 20 deg
        ( (rdf_triple(knowrob:temperatureOfObject, HeatSource, TempHeatSource),
          strip_literal_type(TempHeatSource, THSatom),
          term_to_atom(THSterm, THSatom)) ;
          THSterm=20 ),!,

        TempBaked = 120,
        THSterm > TempBaked,
        THSterm > TDterm,


  % % % % % % % % % % % % %
  % Relations (proportionality, newly generated instances like gas or a flow rate)
  % none


  % % % % % % % % % % % % %
  % Influences of the process on the individuals
  rdf_instance_from_class(knowrob:'Baking-Hardening', Ev),
  rdf_instance_from_class(knowrob:'Baked', Res),

  rdf_assert(Ev, knowrob:inputsDestroyed, Dough),
  rdf_assert(Ev, knowrob:outputsCreated,  Res),

  print(Dough),print(' -> '), print(Res), print('\n'),

  get_timepoint(NOW),
  rdf_assert(Ev, knowrob:startTime, NOW),

  get_timepoint('+2m', THEN),
  rdf_assert(Ev, knowrob:endTime, THEN).




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



