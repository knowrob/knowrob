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

:- module(knowrob_ros_service,
    [
      ros_service/3,
      ros_service_call/3,
      ros_request_encode/2,
      ros_response_decode/2,
      ros_message_conversion/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('http/json')).
:- use_module(library('knowrob/ros')).
:- use_module(library('knowrob/rdfs')).

:- rdf_db:rdf_register_ns(ros, 'http://www.ease-crc.org/ont/ROS.owl#', [keep(true)]).

:- rdf_meta ros_service(r, ?, ?),
            ros_request_encode(r, ?),
            ros_response_decode(+, -),
            ros_service_call(r, r, r).

:- multifile ros_message_conversion/3.

%% ros_service(+Service,?Name,?Path) is det.
%
ros_service(Service,Name,Path) :-
  rdf_has(Service,ros:concretelyImplements,ServiceInterface),
  rdf_has_prolog(Service,ros:hasServiceName,Name),
  rdf_has_prolog(ServiceInterface,ros:hasTypePath,Path).

%% ros_message_slot_type(+Msg,?SlotName,?SlotType) is det.
%
ros_message_slot_type(Msg,SlotName,SlotType) :-
  rdf_has(Msg,dul:realizes,Msg_Type),
  rdf_has(Msg_Type,dul:hasPart,SlotType),
  rdf_has_prolog(SlotType,ros:hasSlotName,SlotName),!.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % call a service
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% ros_service_call(+Service, +Request, +Response) is semidet.
%
ros_service_call(Service, Request, Response) :-
  ros_service(Service, ServiceName, ServicePath),
  ( ros_request_encode(Request, Request_json) ;
    throw(ros_error(ros:'UNGROUNDABLE_REQUEST')) ),
  ( ros_json_wrapper(ServiceName, ServicePath, Request_json, Response_json) ;
    throw(ros_error(ros:'SERVICE_NODE_UNREACHABLE')) ),
  ( ground(Response_json) ; % no response from service
    throw(ros_error(ros:'SERVICE_NODE_UNREACHABLE')) ),
  ( ros_response_decode(Response_json, Response) ;
    throw(ros_error(ros:'UNINTERPRETABLE_REQUEST')) ).

%% ros_request_encode(+Request, -Request_json) is det.
%
ros_request_encode(Request, Request_json) :-
  ros_entity_to_prolog(Request, Request_dict),
  %%%%%%%%%
  %%%%% encode as JSON dict
  %%%%%%%%%
  with_output_to(atom(Request_json), 
    json_write_dict(current_output, Request_dict)
  ).

%% ros_response_decode(+Response_json, +Response) is det.
%
ros_response_decode(Response_json, Response) :-
  %%%%%%%%%
  %%%%% Encode response as list of key-value pairs
  %%%%%%%%%
  atom_to_chars(Response_json,Response_chars),
  open_chars_stream(Response_chars, Response_Stream),
  json_read_dict(Response_Stream, Response_dict),
  dict_pairs(Response_dict,_,Response_pairs),
  %%%%%%%%%
  %%%%% Encode response as list of pairs
  %%%%%%%%%
  forall((
    member(SName-SValue, Response_pairs),
    ros_message_slot_type(Response, SName, DataSlot)),(
    %%%%%%%%%
    %%%%% Create symbolic representation for response field
    %%%%%%%%%
    once((
      rdf_has(DataSlot,dul:hasPart,SlotType),
      ros_type_path(SlotType,SType)
    )),
    owl_create_ros_entity(SType,SValue,Slot),
    rdf_assert(Response,dul:hasPart,Slot),
    rdf_assert(Slot,dul:realizes,DataSlot)
    %,
    %forall(
      %rdf_has(DataSlot,dul:isClassifiedBy,R),
      %classify_message_value(Slot,R)
    %)
  )).

%classify_message_value(Slot,R) :-
  %( rdfs_individual_of(Slot,ros:'PrimitiveValue') ;
    %rdfs_individual_of(Slot,ros:'PrimitiveArray') ), !,
  %rdf_has(Slot, dul:hasRegion, Region),
  %rdf_assert(Region,dul:isClassifiedBy,R).
%classify_message_value(Slot,R) :-
  %% handle message with region representation
  %rdfs_individual_of(Slot,ros:'Message'),
  %rdf_has(Slot,dul:hasRegion,Region),!,
  %rdf_assert(Region,dul:isClassifiedBy,R).
%classify_message_value(Slot,R) :-
  %rdf_assert(Slot,dul:isClassifiedBy,R).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % geometry_msgs/Transform
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

ros_message_conversion('geometry_msgs/Transform',
                       get_transform_dict,
                       get_transform_region).

get_transform_dict(Message,_{
      translation: ['geometry_msgs/Vector3',
          _{x:['float64',Tx], y:['float64',Ty], z:['float64',Tz]}],
      rotation:    ['geometry_msgs/Quaternion',
          _{x:['float64',Qx], y:['float64',Qy], z:['float64',Qz], w:['float64',Qw]}]
  }) :-
  rdf_has(Message,dul:hasRegion,Region),
  rdf_has_prolog(Region, knowrob:translation, [Tx,Ty,Tz]),
  rdf_has_prolog(Region, knowrob:quaternion,  [Qx,Qy,Qz,Qw]).

get_transform_region(_{
      translation: _{x:Tx, y:Ty, z:Tz},
      rotation:    _{x:Qx, y:Qy, z:Qz, w:Qw}
  }, Region) :-
  rdf_instance_from_class(knowrob:'Pose',Region),
  rdf_assert_prolog(Region, knowrob:translation, [Tx,Ty,Tz]),
  rdf_assert_prolog(Region, knowrob:quaternion,  [Qx,Qy,Qz,Qw]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % OWL to ROS message dict
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

ros_message_to_prolog(_, Msg, Msg_dict) :-
  rdf_has(Msg,dul:realizes,Msg_Type),
  findall(SName-[SType,SValue], (
    %%%%%%%%%
    %%%%% For each slot of the message type
    %%%%%%%%%
    rdf_has(Msg_Type,dul:hasPart,DataSlot),
    rdf_has_prolog(DataSlot, ros:hasSlotName, SName),
    once((
      rdf_has(DataSlot,dul:hasPart,SlotType),
      ros_type_path(SlotType,SType)
    )),
    %%%%%%%%%
    %%%%% Infer the value of the slot
    %%%%%%%%%
    once((
      rdf_has(Msg, dul:hasPart, Filler),
      rdf_has(Filler, dul:realizes, DataSlot)
    )),
    ros_entity_to_prolog(Filler, SValue)
  ), Pairs),
  % create a dict 'msg{key1:[type1,value1],...}'
  dict_pairs(Msg_dict,msg,Pairs).

ros_entity_to_prolog(Msg, Msg_dict) :-
  rdfs_individual_of(Msg,ros:'Message'),!,
  rdf_has(Msg,dul:realizes,Msg_Type),
  rdf_has_prolog(Msg_Type,ros:hasTypePath,TypePath),
  once((
    % first check if message is represented as region
    ros_message_conversion(TypePath, GetDictGoal, _),
    call(GetDictGoal, Msg, Msg_dict)) ; 
    % elese traverse message parts
    ros_message_to_prolog(TypePath, Msg, Msg_dict)).

ros_entity_to_prolog(PrimitiveValue_owl, PrimitiveValue_pl) :-
  ( rdfs_individual_of(PrimitiveValue_owl,ros:'PrimitiveValue') ;
    rdfs_individual_of(PrimitiveValue_owl,ros:'PrimitiveArray') ),!,
  rdf_has(PrimitiveValue_owl,dul:hasRegion,Region),
  once((
    rdf_has_prolog(Region,dul:hasRegionDataValue,PrimitiveValue_pl);
    PrimitiveValue_pl = Region )).

ros_entity_to_prolog(Array, List) :-
  rdfs_individual_of(Array,ros:'Array'),!,
  rdf_has(Array, dul:concretelyExpresses, Collection),
  ros_entity_to_prolog(Collection, List).

ros_entity_to_prolog(Collection, List) :-
  rdfs_individual_of(Collection,dul:'Collection'),!,
  ( rdf_has(Collection, ease:firstMember, First) ->
    owl_sequence(First, List) ; 
    findall(M, rdf_has(Collection,dul:hasMember, M), Xs) ),
  findall(X, (
    member(Iri,Xs),
    ros_entity_to_prolog(Iri,X)),
    List).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % ROS message dict to OWL
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

owl_create_ros_entity(Msg_TypePath,Msg_dict,Msg_owl) :-
  is_dict(Msg_dict),!,
  rdf_instance_from_class(ros:'Message',Msg_owl),
  ( rdf_has(Msg_Type,ros:hasTypePath,literal(type(_,Msg_TypePath))) ->
    rdf_assert(Msg_owl,dul:realizes,Msg_Type) ; true ),
  once((
    ros_message_conversion(Msg_TypePath, _, GetRegionGoal),
    call(GetRegionGoal,Msg_dict,Region_owl),
    rdf_assert(Msg_owl,dul:hasRegion,Region_owl)
  );(
    dict_pairs(Msg_dict,_,Msg_pairs),
    forall(member(SName-[SType,SValue], Msg_pairs),(
      owl_create_ros_entity(SType,SValue,Val_owl),
      rdf_assert_prolog(Val_owl,ease:hasNameString,SName),
      rdf_assert(Msg_owl,dul:hasPart,Val_owl)
    ))
  )).

owl_create_ros_entity(Type,Value,PrimitiveValue) :-
  ros_primitive_type(Type, XSDType),!,
  owl_create_atomic_region(XSDType, Value, Region),
  rdf_instance_from_class(ros:'PrimitiveValue',PrimitiveValue),
  rdf_assert(PrimitiveValue,dul:hasRegion,Region).

owl_create_ros_entity(Array_type,Val_list,PrimitiveArray) :-
  is_list(Val_list),
  term_to_atom(array(Type),Array_type),
  ros_primitive_type(Type, _),!,
  ros_array_type(Type, ArrayType),
  % list to atom
  findall(A, (member(X,Val_list), term_to_atom(X,A)), Atoms),
  atomic_list_concat(Atoms, ' ', ArrayData),
  % create symbols
  rdf_instance_from_class(ros:'PrimitiveArray',PrimitiveArray),
  rdf_instance_from_class(dul:'Region',Region),
  rdf_assert(Region,dul:hasRegionDataValue,
             literal(type(ArrayType,ArrayData))),
  rdf_assert(PrimitiveArray,dul:hasRegion,Region).

owl_create_ros_entity(ArrayTypePath,Val_list,MessageArray) :-
  is_list(Val_list),
  term_to_atom(array(Type),ArrayTypePath),
  ros_type_path(ArrayType,ArrayTypePath),
  findall(Val_owl, (
    member(X,Val_list),
    owl_create_ros_entity(Type,X,Val_owl)),
    Msg_list),
  owl_create_ordered_collection(Msg_list, Collection),
  rdf_instance_from_class(ros:'MessageArray',MessageArray),
  rdf_assert(MessageArray, dul:realizes, ArrayType),
  rdf_assert(MessageArray, dul:concretelyExpresses, Collection).
