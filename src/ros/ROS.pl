:- module('knowrob/model/ROSOWL', []).

%:- module('knowrob/model/ROSOWL',
%    [
%      ros_type_path/2,
%      ros_primitive_type/2,
%      ros_array_type/2,
%      ros_service/3,
%      rosowl_service_call/3,
%      ros_request_encode/2,
%      ros_response_decode/2,
%      ros_message_conversion/3,
%      create_ros_request/5
%    ]).
%
%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/owl')).
%:- use_module(library('http/json')).
%
%:- use_module(library('knowrob/comp/rdf_data'), [kb_rdf_pl/3]).
%
%:- rdf_meta ros_type_path(r,?),
%            ros_primitive_type(?,r),
%            ros_service(r, ?, ?),
%            ros_request_encode(r, ?),
%            ros_response_decode(+, -),
%            rosowl_service_call(r, r, r),
%            create_ros_request(r,t,t,r,r).
%
%:- multifile ros_message_conversion/3.
%
%%% ros_type_path(+MessageType,?TypePath) is det.
%%
%ros_type_path(MessageType,TypePath) :-
%  kb_type_of(MessageType,ros:'MessageType'),
%  kb_triple(MessageType,ros:hasTypePath,TypePath),!.
%ros_type_path(PrimitiveType,TypePath) :-
%  kb_type_of(PrimitiveType,ros:'PrimitiveType'),
%  rdf_split_url(_, TypePath, PrimitiveType),!.
%ros_type_path(ArrayType, ArrayType_atom) :-
%  kb_type_of(ArrayType,ros:'ArrayType'),
%  once((
%    kb_triple(ArrayType,dul:hasPart,X),
%    ros_type_path(X,T))),
%  term_to_atom(array(T), ArrayType_atom),!.
%
%%% ros_primitive_type(?ROS_type, ?RDF_type) is det.
%%
%% A mapping between ROS and RDF types.
%%
%ros_primitive_type('bool',    'http://www.w3.org/2001/XMLSchema#boolean').
%ros_primitive_type('float32', 'http://www.w3.org/2001/XMLSchema#float').
%ros_primitive_type('float64', 'http://www.w3.org/2001/XMLSchema#double').
%ros_primitive_type('int8',    'http://www.w3.org/2001/XMLSchema#byte').
%ros_primitive_type('int16',   'http://www.w3.org/2001/XMLSchema#short').
%ros_primitive_type('int32',   'http://www.w3.org/2001/XMLSchema#int').
%ros_primitive_type('int64',   'http://www.w3.org/2001/XMLSchema#long').
%ros_primitive_type('uint8',   'http://www.w3.org/2001/XMLSchema#unsignedByte').
%ros_primitive_type('uint16',  'http://www.w3.org/2001/XMLSchema#unsignedShort').
%ros_primitive_type('uint32',  'http://www.w3.org/2001/XMLSchema#unsignedInt').
%ros_primitive_type('uint64',  'http://www.w3.org/2001/XMLSchema#unsignedLong').
%ros_primitive_type('string',  'http://www.w3.org/2001/XMLSchema#string').
%% TODO support duration and time
%%ros_primitive_type('duration',_).
%%ros_primitive_type('time',xsd:dateTime).
%
%ros_array_type('bool',    'http://www.ease-crc.org/ont/EASE.owl#array_boolean').
%ros_array_type('float32', 'http://www.ease-crc.org/ont/EASE.owl#array_float').
%ros_array_type('float64', 'http://www.ease-crc.org/ont/EASE.owl#array_double').
%ros_array_type('int8',    'http://www.ease-crc.org/ont/EASE.owl#array_int').
%ros_array_type('int16',   'http://www.ease-crc.org/ont/EASE.owl#array_int').
%ros_array_type('int32',   'http://www.ease-crc.org/ont/EASE.owl#array_int').
%ros_array_type('int64',   'http://www.ease-crc.org/ont/EASE.owl#array_int').
%ros_array_type('uint8',   'http://www.ease-crc.org/ont/EASE.owl#array_uint').
%ros_array_type('uint16',  'http://www.ease-crc.org/ont/EASE.owl#array_uint').
%ros_array_type('uint32',  'http://www.ease-crc.org/ont/EASE.owl#array_uint').
%ros_array_type('uint64',  'http://www.ease-crc.org/ont/EASE.owl#array_uint').
%ros_array_type('string',  'http://www.ease-crc.org/ont/EASE.owl#array_string').
%
%%% ros_service(+Service,?Name,?Path) is det.
%%
%ros_service(Service,Name,Path) :-
%  kb_triple(Service,ros:concretelyImplements,ServiceInterface),
%  kb_triple(Service,ros:hasServiceName,Name),
%  kb_triple(ServiceInterface,ros:hasTypePath,Path).
%
%%% ros_message_slot_type(+Msg,?SlotName,?SlotType) is det.
%%
%ros_message_slot_type(Msg,SlotName,SlotType) :-
%  kb_triple(Msg,dul:realizes,Msg_Type),
%  kb_triple(Msg_Type,dul:hasPart,SlotType),
%  kb_triple(SlotType,ros:hasSlotName,SlotName),!.
%
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% % % % % % % % % % % call a service
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%
%%% rosowl_service_call(+Service, +Request, +Response) is semidet.
%%
%rosowl_service_call(Service, Request, Response) :-
%  ros_service(Service, ServiceName, ServicePath),
%  ( ros_request_encode(Request, Request_json) ;
%    throw(ros_error(ros:'SerializationFailure')) ),
%  ( ros_json_service_call(_{
%        service_path: ServicePath,
%        service_name: ServiceName,
%        json_data: Request_json
%  }, Response_json) ;
%    throw(ros_error(ros:'ServiceNodeUnreachable')) ),
%  ( ground(Response_json) ; % no response from service
%    throw(ros_error(ros:'ServiceNodeUnreachable')) ),
%  ( ros_response_decode(Response_json, Response) ;
%    throw(ros_error(ros:'SerializationFailure')) ).
%
%%% ros_request_encode(+Request, -Request_json) is det.
%%
%ros_request_encode(Request, Request_json) :-
%  ros_entity_to_prolog(Request, Request_dict),!,
%  %%%%%%%%%
%  %%%%% encode as JSON dict
%  %%%%%%%%%
%  with_output_to(atom(Request_json), 
%    json_write_dict(current_output, Request_dict)
%  ).
%
%%% ros_response_decode(+Response_json, +Response) is det.
%%
%ros_response_decode(Response_json, Response) :-
%  %%%%%%%%%
%  %%%%% Encode response as list of key-value pairs
%  %%%%%%%%%
%  atom_to_chars(Response_json,Response_chars),
%  open_chars_stream(Response_chars, Response_Stream),
%  json_read_dict(Response_Stream, Response_dict),
%  dict_pairs(Response_dict,_,Response_pairs),
%  %%%%%%%%%
%  %%%%% Encode response as list of pairs
%  %%%%%%%%%
%  forall((
%    member(SName-SValue, Response_pairs),
%    ros_message_slot_type(Response, SName, DataSlot)),(
%    %%%%%%%%%
%    %%%%% Create symbolic representation for response field
%    %%%%%%%%%
%    once((
%      kb_triple(DataSlot,dul:hasPart,SlotType),
%      ros_type_path(SlotType,SType)
%    )),
%    owl_create_ros_entity(SType,SValue,Slot),
%    kb_assert(Response,dul:hasPart,Slot),
%    kb_assert(Slot,dul:realizes,DataSlot)
%  )).
%
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% % % % % % % % % % % geometry_msgs/Transform
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%
%ros_message_conversion('geometry_msgs/Transform',
%                       get_transform_dict,
%                       get_transform_region).
%
%get_transform_dict(Message,_{
%      translation: ['geometry_msgs/Vector3',
%          _{x:['float64',Tx], y:['float64',Ty], z:['float64',Tz]}],
%      rotation:    ['geometry_msgs/Quaternion',
%          _{x:['float64',Qx], y:['float64',Qy], z:['float64',Qz], w:['float64',Qw]}]
%  }) :-
%  kb_triple(Message,dul:hasRegion,Region),
%  kb_triple(Region, knowrob:translation, [Tx,Ty,Tz]),
%  kb_triple(Region, knowrob:quaternion,  [Qx,Qy,Qz,Qw]).
%
%get_transform_region(_{
%      translation: _{x:Tx, y:Ty, z:Tz},
%      rotation:    _{x:Qx, y:Qy, z:Qz, w:Qw}
%  }, Region) :-
%  kb_create(knowrob:'Pose',Region),
%  kb_assert(Region, knowrob:translation, [Tx,Ty,Tz]),
%  kb_assert(Region, knowrob:quaternion,  [Qx,Qy,Qz,Qw]).
%
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% % % % % % % % % % % OWL to ROS message dict
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%
%ros_message_to_prolog(_, Msg, Msg_dict) :-
%  kb_triple(Msg,dul:realizes,Msg_Type),
%  findall(SName-[SType,SValue], (
%    %%%%%%%%%
%    %%%%% For each slot of the message type
%    %%%%%%%%%
%    kb_triple(Msg_Type,dul:hasPart,DataSlot),
%    once((
%      kb_triple(DataSlot, ros:hasSlotName, SName),
%      kb_triple(DataSlot,dul:hasPart,SlotType),
%      ros_type_path(SlotType,SType)
%    )),
%    %%%%%%%%%
%    %%%%% Infer the value of the slot
%    %%%%%%%%%
%    once((
%      kb_triple(Msg, dul:hasPart, Filler),
%      kb_triple(Filler, dul:realizes, DataSlot),
%      ros_entity_to_prolog(Filler, SValue)
%    ))
%  ), Pairs),
%  % create a dict 'msg{key1:[type1,value1],...}'
%  dict_pairs(Msg_dict,msg,Pairs).
%
%ros_entity_to_prolog(Msg, Msg_dict) :-
%  kb_type_of(Msg,ros:'Message'),!,
%  kb_triple(Msg,dul:realizes,Msg_Type),
%  kb_triple(Msg_Type,ros:hasTypePath,TypePath),
%  once(((
%    % first check if message is represented as region
%    ros_message_conversion(TypePath, GetDictGoal, _),
%    call(GetDictGoal, Msg, Msg_dict)) ; 
%    % elese traverse message parts
%    ros_message_to_prolog(TypePath, Msg, Msg_dict))).
%
%ros_entity_to_prolog(PrimitiveValue_owl, PrimitiveValue_pl) :-
%  ( kb_type_of(PrimitiveValue_owl,ros:'PrimitiveValue') ;
%    kb_type_of(PrimitiveValue_owl,ros:'PrimitiveArray') ),!,
%  kb_triple(PrimitiveValue_owl,dul:hasRegion,Region),
%  (( rdf_has(Region,dul:hasRegionDataValue,X),
%     kb_rdf_pl(dul:hasRegionDataValue,X,PrimitiveValue_pl)
%  ); PrimitiveValue_pl = Region
%  ), !.
%
%ros_entity_to_prolog(Array, List) :-
%  kb_type_of(Array,ros:'Array'),!,
%  kb_triple(Array, dul:concretelyExpresses, Collection),
%  ros_entity_to_prolog(Collection, List).
%
%ros_entity_to_prolog(Collection, List) :-
%  kb_type_of(Collection,dul:'Collection'),!,
%  ( kb_triple(Collection, soma:firstMember, First) ->
%    owl_sequence(First, List) ; 
%    findall(M, kb_triple(Collection,dul:hasMember, M), Xs) ),
%  findall(X, (
%    member(Iri,Xs),
%    ros_entity_to_prolog(Iri,X)),
%    List).
%
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% % % % % % % % % % % ROS message dict to OWL
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%
%owl_create_ros_entity(Msg_TypePath,Msg_dict,Msg_owl) :-
%  is_dict(Msg_dict),!,
%  kb_create(ros:'Message',Msg_owl),
%  ( kb_triple(Msg_Type,ros:hasTypePath,Msg_TypePath) ->
%    kb_assert(Msg_owl,dul:realizes,Msg_Type) ; true ),
%  once((
%    ros_message_conversion(Msg_TypePath, _, GetRegionGoal),
%    call(GetRegionGoal,Msg_dict,Region_owl),
%    kb_assert(Msg_owl,dul:hasRegion,Region_owl)
%  );(
%    dict_pairs(Msg_dict,_,Msg_pairs),
%    forall(member(SName-[SType,SValue], Msg_pairs),(
%      owl_create_ros_entity(SType,SValue,Val_owl),
%      kb_assert(Val_owl,soma:hasNameString,SName),
%      kb_assert(Msg_owl,dul:hasPart,Val_owl)
%    ))
%  )).
%
%owl_create_ros_entity(Type,Value,PrimitiveValue) :-
%  ros_primitive_type(Type, XSDType),!,
%  kb_create(dul:'Region',Region),
%  ( atom(Value) -> Atom = Value ; term_to_atom(Value,Atom) ),
%  kb_assert(Region,dul:hasRegionDataValue,
%            literal(type(XSDType,Atom))),
%  kb_create(ros:'PrimitiveValue',PrimitiveValue),
%  kb_assert(PrimitiveValue,dul:hasRegion,Region).
%
%owl_create_ros_entity(Array_type,Val_list,PrimitiveArray) :-
%  is_list(Val_list),
%  term_to_atom(array(Type),Array_type),
%  ros_primitive_type(Type, _),!,
%  ros_array_type(Type, ArrayType),
%  % list to atom
%  findall(A, (member(X,Val_list), term_to_atom(X,A)), Atoms),
%  atomic_list_concat(Atoms, ' ', ArrayData),
%  % create symbols
%  kb_create(ros:'PrimitiveArray',PrimitiveArray),
%  kb_create(dul:'Region',Region),
%  kb_assert(Region,dul:hasRegionDataValue,
%             literal(type(ArrayType,ArrayData))),
%  kb_assert(PrimitiveArray,dul:hasRegion,Region).
%
%owl_create_ros_entity(ArrayTypePath,Val_list,MessageArray) :-
%  is_list(Val_list),
%  term_to_atom(array(Type),ArrayTypePath),
%  ros_type_path(ArrayType,ArrayTypePath),
%  findall(Val_owl, (
%    member(X,Val_list),
%    owl_create_ros_entity(Type,X,Val_owl)),
%    Msgs),
%  kb_create(dul:'Collection',Collection),
%  forall( member(X,Msgs), kb_assert(Collection,dul:hasMember,X) ),
%  kb_create(ros:'MessageArray',MessageArray),
%  kb_assert(MessageArray, dul:realizes, ArrayType),
%  kb_assert(MessageArray, dul:concretelyExpresses, Collection).
%
%%% create_ros_request(Action,InputDict,ReqType,Request)
%%
%%
%create_ros_request(Action,InputDict,ActionDict,ReqType,Request) :-
%  %%%%%%%%%
%  %%%%% Create request message
%  kb_create(ros:'Message',Request),
%  kb_assert(Request,dul:realizes,ReqType),
%  kb_assert(Action,ros:hasRequest,Request),
%  %%%%%%%%%
%  %%%%% Set request slots
%  forall(kb_triple(ReqType,dul:hasPart,DataSlot), once((
%    % for each data slot, find participant of the 
%    % action that is classified by the same parameter or role
%    get_dict(Concept,InputDict,Filler),
%    get_dict(Concept,ActionDict,DataSlot),
%    %kb_triple(DataSlot,dul:hasPart,SlotType),
%    create_ros_message_slot(DataSlot,Filler,Slot),
%    kb_assert(Request,dul:hasPart,Slot),
%    kb_assert(Slot,dul:realizes,DataSlot)
%  ))).
%
%%%
%create_ros_message_slot(SlotType, Region, Slot) :-
%  rdfs_individual_of(SlotType,ros:'PrimitiveSlot'),!,
%  rdfs_individual_of(Region,dul:'Region'),
%  kb_create(ros:'PrimitiveValue',Slot),
%  kb_assert(Slot, dul:hasRegion, Region).
%
%create_ros_message_slot(SlotType, Region, Slot) :-
%  rdfs_individual_of(SlotType,ros:'ArraySlot'),
%  rdfs_individual_of(Region,dul:'Region'),!,
%  kb_create(ros:'PrimitiveArray',Slot),
%  kb_assert(Slot, dul:hasRegion, Region).
%
%create_ros_message_slot(SlotType, Array, Array) :-
%  rdfs_individual_of(SlotType,ros:'ArraySlot'),!,
%  rdfs_individual_of(Array,ros:'Array').
%
%create_ros_message_slot(SlotType, Region, Message) :-
%  rdfs_individual_of(SlotType,ros:'MessageSlot'),
%  rdfs_individual_of(Region,dul:'Region'),
%  kb_triple(SlotType,dul:hasPart,MessageType),!,
%  kb_create(ros:'Message',Message),
%  kb_assert(Message,dul:realizes,MessageType),
%  kb_assert(Message,dul:hasRegion,Region).
%
%create_ros_message_slot(SlotType, Message, Message) :-
%  rdfs_individual_of(SlotType,ros:'MessageSlot'),!,
%  rdfs_individual_of(Message,ros:'Message').
