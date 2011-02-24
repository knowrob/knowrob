
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% GENERIC PROCESS SCHEMA
%
% The modeling of processes largely follows the approach proposed in the
% Qualitative Process Theory (QP) [1].
%
% A process description mainly consists of the following fields which may be empty:
% - a set of individuals that are changed by the process
% - some pre-conditions for the process to be applicable at all (which are outside
%   of QP, like one object being thermically connected to another one)
% - quantity conditions that decide if the process is active or not, e.g. a heat
%   difference between two objects which is required for a heat flow to start
% - proportionality relations that can be used to reason about combinations and
%   consequences of processes
% - the influence of the individuals, e.g. objects that are consumed by the process,
%   others that appear, or object states or properties that are changed
%
% [1] Qualitative process theory, KD Forbus, Artificial intelligence, 1984
%


:- module(processes,
    [
      process/0
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Dough becomes Baked during a baking process
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



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% cooling devices become cold, heating devices hot when being switched on

process :-
  % % % % % % % % % % % % %
  % Individuals changed in the process
  (owl_individual_of(Device, knowrob:'HeatingDevice'); owl_individual_of(Device, knowrob:'CoolingDevice')),

  % % % % % % % % % % % % %
  % Preconditions (outside of QP)
  owl_has(Device, knowrob:stateOfObject, knowrob:'DeviceStateOn'),

  % % % % % % % % % % % % %
  % QuantityConditions (prerequisites for the process to be active, inside of QP)
  % none

  % % % % % % % % % % % % %
  % Relations (proportionality, newly generated instances like gas or a flow rate)
  % none

  % % % % % % % % % % % % %
  % Influences of the process on the individuals

  % simplified view: working temperature reached immediately after switching on the device
  owl_has(Device, knowrob:workingTemperature, WorkingTemp),!,
  rdf_assert(Device, knowrob:temperatureOfObject, WorkingTemp).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% heating and cooling devices come to room temperature when being switched off

process :-
  % % % % % % % % % % % % %
  % Individuals changed in the process
  (owl_individual_of(Device, knowrob:'HeatingDevice'); owl_individual_of(Device, knowrob:'CoolingDevice')),

  % % % % % % % % % % % % %
  % Preconditions (outside of QP)
  owl_has(Device, knowrob:stateOfObject, knowrob:'DeviceStateOff'),!,

  % % % % % % % % % % % % %
  % QuantityConditions (prerequisites for the process to be active, inside of QP)
  % none

  % % % % % % % % % % % % %
  % Relations (proportionality, newly generated instances like gas or a flow rate)
  % none

  % % % % % % % % % % % % %
  % Influences of the process on the individuals

  % simplified view: room temperature reached immediately after switching off the device
  rdf_assert(Device, knowrob:temperatureOfObject, literal(type('http://www.w3.org/2001/XMLSchema#integer', '20'))).




