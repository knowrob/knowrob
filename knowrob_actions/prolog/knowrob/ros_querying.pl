/*
  Copyright (C) 2019 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(ros_querying,
    [
      ros_querying/1
    ]).
/** <module> The execution of ROS querying actions.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/action_execution')).

:- rdf_meta ros_querying(r).

action_execution:action_registry('http://www.ease-crc.org/ont/ROS.owl#ROSQuerying', ros_querying).

%% ros_querying(+Action) is semidet.
%
%
ros_querying(Action) :-
  %%%%%%%%%
  %%%%% Find ServiceInterface participant.
  action_call_or_failure(Action, (
      action_participant_type(Action,ServiceInterface,ros:'ServiceInterface')
    ),
    knowrob:'ACTION_INPUT_MISSING',
    'no service interface participant'
  ),!,
  %%%%%%%%%
  %%%%% Find the ROS service (i.e. some object that concretely realizes
  %%%%% the interface.
  action_call_or_failure(Action, (
      rdf_has(Service,dul:concretelyImplements,ServiceInterface)
    ),
    ros:'SERVICE_NODE_UNREACHABLE',
    'OWL ROS Service missing'
  ),
  action_call_or_failure(Action, (
      owl_has(ServiceInterface,ros:hasResponseType,ResType),
      owl_has(ServiceInterface,ros:hasRequestType,ReqType),
      rdf_assert(Action,ease:isAnsweredBy,Service)
    ),
    knowrob:'ACTION_MODEL_ERROR',
    'request or response type missing'
  ),
  action_call_or_failure(Action, (
    forall(rdf_has(ReqType,dul:hasPart,DataSlot), 
      once(action_filler_for(Action,DataSlot,_)))
    ),
    knowrob:'ACTION_INPUT_MISSING',
    'request slot(s) missing'
  ),
  %%%%%%%%%
  %%%%% Create request and response message
  rdf_instance_from_class(ros:'Message',Request),
  rdf_instance_from_class(ros:'Message',Response),
  rdf_assert(Request,dul:realizes,ReqType),
  rdf_assert(Response,dul:realizes,ResType),
  rdf_assert(Action,ros:hasRequest,Request),
  rdf_assert(Action,ros:hasResponse,Response),
  %%%%%%%%%
  %%%%% Set request slots
  forall(rdf_has(ReqType,dul:hasPart,DataSlot), once((
    % for each data slot, find participant of the 
    % action that is classified by the same parameter or role
    action_filler_for(Action,DataSlot,Filler),
    rdf_has(DataSlot,dul:hasPart,SlotType),
    create_ros_message_slot(SlotType,Filler,Slot),
    rdf_assert(Request,dul:hasPart,Slot),
    rdf_assert(Slot,dul:realizes,DataSlot)
  ))),
  %%%%%%%%%
  %%%%% Call the service
  catch(
    ros_service_call(Service, Request, Response),
    ros_error(Error),
    throw(action_failure(Action, Error, 'ROS service call failed'))
  ),
  %%%%%%%%%
  %%%%% Assign roles of response slots
  forall(rdf_has(ResType,dul:hasPart,DataSlot), (
    rdf_has(Response,dul:hasPart,Slot),
    rdf_has(Slot,dul:realizes,DataSlot),
    ((rdfs_individual_of(Slot,ros:'PrimitiveValue');
      rdfs_individual_of(Slot,ros:'PrimitiveArray'))-> 
      rdf_has(Slot,dul:hasRegion,X) ; X=Slot ),
    ( rdfs_individual_of(DataSlot,ros:'StatusSlot') ->
      % handle dedicated status field
      ros_querying_set_status(Action,DataSlot,X) ;
      % handle roles and parameters
      action_add_filler(Action,X)
    )
  )).

% update action status according to
% status field of response message
ros_querying_set_status(Action,DataSlot,SlotRegion) :-
  % get ROS status code
  rdf_has_prolog(SlotRegion,dul:hasDataValue,StatusCode),
  % find mapping into region
  rdf_has(DataSlot,ros:hasRegionMapping,Mapping),
  rdf_has_prolog(Mapping,dul:hasDataValue,StatusCode),
  rdf_has(       Mapping,dul:hasRegion,Status),
  % update the status
  set_action_status(Action,Status), !.
ros_querying_set_status(Action,_,SlotRegion) :-
  rdf_has_prolog(SlotRegion,dul:hasDataValue,StatusCode),
  rdfs_split_url(_,AName,Action),
  writef('[WARN] Action %w has unknown status %w.', [AName,StatusCode]).

%%
create_ros_message_slot(SlotType, Region, Slot) :-
  rdfs_individual_of(SlotType,ros:'PrimitiveSlot'),!,
  rdfs_individual_of(Region,ros:'Region'),
  rdf_instance_from_class(ros:'PrimitiveValue',Slot),
  rdf_assert(Slot, dul:hasRegion, Region).
create_ros_message_slot(SlotType, Region, Slot) :-
  rdfs_individual_of(SlotType,ros:'ArraySlot'),!,
  rdfs_individual_of(Region,ros:'Region'),
  rdf_instance_from_class(ros:'PrimitiveArray',Slot),
  rdf_assert(Slot, dul:hasRegion, Region).
create_ros_message_slot(SlotType, Array, Array) :-
  rdfs_individual_of(SlotType,ros:'ArraySlot'),!,
  rdfs_individual_of(Array,ros:'Array').
create_ros_message_slot(SlotType, Message, Message) :-
  rdfs_individual_of(SlotType,ros:'MessageSlot'),!,
  rdfs_individual_of(Message,ros:'Message').
