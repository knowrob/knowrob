
:- module(action_effects,
    [
      project_action_effects/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).



:- rdf_db:rdf_register_ns(knowrob,       'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://ias.cs.tum.edu/kb/object-change.owl#', [keep(true)]).

:-  rdf_meta
    project_action_effects(r),
    unlink_object(r),
    remove_object_properties(r,r).


% utility predicate: remove reference to an object (e.g. if it has been destroyed)
%
unlink_object(Obj) :-
  ((findall(Prop, (rdfs_subproperty_of(Prop, knowrob:topologicalRelations)), Props),
    findall(P,    (member(P, Props), rdf_retractall(Obj, P, _)), _),!) ; true,!),
  ((findall(Prop, (rdfs_subproperty_of(Prop, knowrob:orientation)), Props),
    findall(P,    (member(P, Props), rdf_retractall(Obj, P, _)), _),!) ; true,!).

% utility predicate: remove all assertions of sub-properties of Property from Obj
remove_object_properties(Obj, Property) :-
(findall(Prop, (rdfs_subproperty_of(Prop, Property)), Props),
   findall(P,    (member(P, Props), rdf_retractall(Obj, P, _)), _)),!.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Joining substances
%


% % % % % % % % % % % % % % % %
% Mixing baking mix to a dough
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Mixing'),
  \+ owl_has(Action, knowrob:outputsCreated, _),

  % at least one objectActedOn is a MixForBakedGoods or WheatFlour
  owl_has(Action, knowrob:objectActedOn, Mix),
  (owl_individual_of(Mix, knowrob:'MixForBakedGoods');
   owl_individual_of(Mix, knowrob:'WheatFlour') ;
   owl_individual_of(Mix, knowrob:'Dough') ),

  findall(Obj, owl_has(Action, knowrob:objectActedOn, Obj), Objs), !,

  % new objects: Dough is created, things are added to Dough
  rdf_instance_from_class(knowrob:'Dough', knowrob_projection, Dough),
  rdf_assert(Action, knowrob:objectAddedTo,  Dough, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Dough, knowrob_projection),

  % new relations
  findall(O, (member(O, Objs),
              rdf_assert(Action, knowrob:thingIncorporated, O, knowrob_projection),
              print(O),print(' added to -> '), print(Dough), print('\n') ), _).



% % % % % % % % % % % % % % % %
% Adding something to something else
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Incorporation-Physical'),
  \+ owl_individual_of(Action, knowrob:'Mixing'),
  \+ owl_has(Action, knowrob:'', _).  % TODO: implement!





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Heating up and cooking
%


% % % % % % % % % % % % % % % %
% Heat up food
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'HeatingFood'),
  \+ owl_has(Action, knowrob:'', _).  % TODO: implement!

% temperature: 100 degree?



% % % % % % % % % % % % % % % %
% Boil food
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'BoilingFood'),
  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_individual_of(Obj, knowrob:'Boiled'),!,

  rdf_assert(Obj, rdf:type, knowrob:'Boiled', knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Obj, knowrob_projection),

  rdf_assert(Obj, knowrob:temperatureOfObject, literal(type('http://www.w3.org/2001/XMLSchema#integer', '100')), knowrob_projection),

  print(Obj),print(' boiled '), print('\n').




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Separation actions (cutting, cracking, chopping)
%


% % % % % % % % % % % % % % % %
% Chopping something into small pieces
% project_action_effects(Action) :-
%
%   owl_individual_of(Action, knowrob:'ChoppingSomething'),
%   \+ owl_has(Action, knowrob:'', _).  % TODO: implement!


% % % % % % % % % % % % % % % %
% Taking something out of a container (sugar, coffee grains)
% project_action_effects(Action) :-
%
%   owl_individual_of(Action, knowrob:'RemovingMaterialFromAContainer'),
%   \+ owl_has(Action, knowrob:'', _).  % TODO: implement!



% % % % % % % % % % % % % % % %
% Cleaning something
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Cleaning'),
  \+ owl_has(Action, knowrob:'objectOfStateChange', _),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'Clean'),!,

  % new relations
  remove_object_properties(Obj, knowrob:'stateOfObject'),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'Clean', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'Clean', knowrob_projection),

  % optionally: fromState
  ((owl_has(Obj, knowrob:'stateOfObject', PreState),
    rdf_assert(Action, knowrob:'fromState', PreState, knowrob_projection)) ; (true)),

  print(Obj),print(' cleaned '), print('\n').



% % % % % % % % % % % % % % % %
% Cutting off a piece
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'CuttingOffAPiece'),
  \+ owl_has(Action, knowrob:outputsCreated, _),


  owl_has(Action, knowrob:objectActedOn, Obj),
  rdf_has(Obj, rdf:type, ObjType),
  ObjType \= 'http://www.w3.org/2002/07/owl#NamedIndividual',!,

  % new objects
  rdf_instance_from_class(ObjType, knowrob_projection, Slice),

  % new relations
  rdf_assert(Action, knowrob:outputsRemaining, Obj, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Slice, knowrob_projection),

  print(Obj),print(' -> '), print(Slice), print('\n').



% % % % % % % % % % % % % % % %
% Cracking an egg
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Cracking'),
  \+ owl_has(Action, knowrob:outputsCreated, _),

  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_individual_of(Obj, knowrob:'Egg-Chickens'),!,

  % new objects
  rdf_instance_from_class(knowrob:'EggShell', knowrob_projection, Shell),
  rdf_instance_from_class(knowrob:'EggYolk-Food', knowrob_projection, Yolk),

  % new relations
  rdf_assert(Action, knowrob:inputsDestroyed, Obj, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Shell, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Yolk, knowrob_projection),

  print(Obj),print(' -> '), print(Shell), print('\n'),
  print(Obj),print(' -> '), print(Yolk), print('\n').






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Moving something to a location
%



% % % % % % % % % % % % % % % %
% Applying something to a surface (e.g. spreading butter)
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'ApplyingSomethingToSurface'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'on-Physical', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').



% % % % % % % % % % % % % % % %
% Putting something onto something
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingOnto'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'on-Physical', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').


% % % % % % % % % % % % % % % %
% Putting something into a container
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingIntoSomething'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'in-ContGeneric', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').




% % % % % % % % % % % % % % % %
% Putting something to some location
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingSomewhere'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_has(Action, knowrob:toLocation, To),!,

  % predict the object to be at the toLocation of the action
  rdf_instance_from_class(knowrob:'ThoughtExperimenting', knowrob_projection, Pred),
  rdf_assert(Pred, knowrob:'objectActedOn', Obj, knowrob_projection),

  get_timepoint(NOW), % TODO: add predicted action duration
  rdf_assert(Pred, knowrob:'startTime', NOW, knowrob_projection),

  ((owl_individual_of(To, knowrob:'RotationMatrix'),!) -> (
    % if toLocation is given as pose matrix, use this one
    rdf_assert(Pred, knowrob:'eventOccursAt', To, knowrob_projection)
   ) ; (
    % otherwise use the knowrob:orientation of the toLocation
    rdf_triple(knowrob:orientation, To, ToPose),
    rdf_assert(Pred, knowrob:'eventOccursAt', ToPose, knowrob_projection)
  )),

  print(Obj),print(' at location '), print(To), print('\n').



% % % % % % % % % % % % % % % %
% Pouring something onto something
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PouringSomethingOnto'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'on-Physical', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Changing device states (turning sth on or off, opening or closing sth)
%

% % % % % % % % % % % % % % % %
% Switching on a device
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOnPoweredDevice'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:'stateOfObject'),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOff', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOn', knowrob_projection),

  print(Obj),print(' switched on '), print('\n').



% % % % % % % % % % % % % % % %
% Switching off a device
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOffPoweredDevice'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOn', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOff', knowrob_projection),

  print(Obj),print(' switched off'), print('\n').



% % % % % % % % % % % % % % % %
% Turning on water (objectActedOn is a tap)
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOnWater'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOff', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOn', knowrob_projection),

  print(Obj),print(' switched on'), print('\n').



% % % % % % % % % % % % % % % %
% Turning off water (objectActedOn is a tap)
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOffWater'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOn', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOff', knowrob_projection),

  print(Obj),print(' switched off'), print('\n').



% % % % % % % % % % % % % % % %
% Opening a container (change state to open)
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'OpeningAContainerArtifact'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateOpen'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateOpen', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'ObjectStateClosed', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'ObjectStateOpen', knowrob_projection),

  print(Obj),print(' opened'), print('\n').



% % % % % % % % % % % % % % % %
% Closing a container (change state to closed)
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'ClosingAContainerArtifact'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateClosed'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateClosed', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'ObjectStateOpen', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'ObjectStateClosed', knowrob_projection),

  print(Obj),print(' opened'), print('\n').



