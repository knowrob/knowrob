
:- module('knowrob/comp/action_execution_ros',
    [
      ros_service_query/4
    ]).
/** <module> The execution of KB querying actions.

@author Daniel Beßler
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('knowrob/lang/ask')).
:- use_module(library('knowrob/lang/tell')).

:- use_module(library('knowrob/model/ROSOWL')).
:- use_module(library('knowrob/model/Event'), [ event_participant/3 ]).
:- use_module(library('knowrob/model/Action'), [ action_add_filler/2 ]).

:- use_module(library('knowrob/comp/action_execution')).

:- rdf_meta ros_service_query(r,t,t,-).

% extend action library
'knowrob/comp/action_execution':action_registry(
  'http://www.ease-crc.org/ont/ROS.owl#ServiceInvokation',
  'knowrob/comp/action_execution_ros':ros_service_query
).

%% ros_service_query(PlanExecution,BindingDict,InputDict,OutputPairs) is semidet.
%
%
ros_service_query(PlanExecution,BindingDict,InputDict,OutputPairs) :-
  kb_triple(PlanExecution,dul:satisfies,ExecutionPlan),
  kb_triple(PlanExecution,dul:includesAction,Action),
  %%%%%%%%%
  %%%%% Find ServiceInterface participant.
  ( event_participant(Action,ServiceInterface,ros:'ServiceInterface') ;
    throw(action_failure(knowrob:'MissingArgument'))
  ),!,
  %%%%%%%%%
  %%%%% Find the ROS service (i.e. some object that concretely realizes
  %%%%% the interface.
  ( get_ros_service(ServiceInterface,ExecutionPlan,Service) ;
    throw(action_failure(ros:'ServiceNodeUnreachable'))
  ),
  ( get_service_interface(ServiceInterface,ReqType,ResType) ;
    throw(action_failure(knowrob:'IncompleteModel'))
  ),
  ( forall(
      kb_triple(ReqType,dul:hasPart,DataSlot), 
      once(action_filler_binding(_:InputDict,DataSlot:BindingDict))) ;
    throw(action_failure(knowrob:'MissingArgument'))
  ),
  kb_assert(Action,ease:isAnsweredBy,Service),
  %%%%%%%%%
  %%%%% Create request and response message
  kb_create(ros:'Message',Response),
  kb_assert(Response,dul:realizes,ResType),
  kb_assert(Action,ros:hasResponse,Response),
  create_ros_request(Action,InputDict,BindingDict,ReqType,Request),
  %%%%%%%%%
  %%%%% Call the service
  catch(
    rosowl_service_call(Service, Request, Response),
    ros_error(Error),
    throw(action_failure(Error))
  ),
  % TODO: special handling for message status field in case
  %        the field value indicates an error.
  %(( get_query_status(Response,Status), is_status_error(Status) ) ->
  %   throw(action_failure(Status)) ; true ),
  %%%%%%%%%
  %%%%% Assign roles of response slots
  findall(R-X, (
    rdf_has(ResType,dul:hasPart,DataSlot),
    rdf_has(Response,dul:hasPart,Slot),
    rdf_has(Slot,dul:realizes,DataSlot),
    get_dict(R, BindingDict, Slot),
    get_slot_filler(Slot,X),
    action_add_filler(Action,X)
  ),OutputPairs).

%%
get_ros_service(ExecutionPlan,ServiceInterface,Service) :-
  kb_triple(Service,ros:concretelyImplements,ServiceInterface),
  ( kb_triple(ExecutionPlan,ros:hasServiceName,SName) ->
    kb_triple(Service,ros:hasServiceName,SName) ; true ).

%%
get_service_interface(ServiceInterface,ReqType,ResType) :-
  kb_triple(ServiceInterface,ros:hasResponseType,ResType),
  kb_triple(ServiceInterface,ros:hasRequestType,ReqType).

%%
get_query_status(Msg,Status) :-
  kb_triple(Msg,dul:hasPart,Slot),
  kb_triple(Slot,dul:realizes,DataSlot),
  kb_type_of(DataSlot,ros:'StatusSlot'),
  get_slot_filler(Slot,Status),!.

%%
get_slot_filler(Slot,Obj) :-
  ( kb_type_of(Slot,ros:'PrimitiveValue') ;
    kb_type_of(Slot,ros:'PrimitiveArray') ), !,
  kb_triple(Slot, dul:hasRegion, Obj).

get_slot_filler(Slot,Obj) :-
  % a message with a region value
  kb_type_of(Slot,ros:'Message'),
  kb_triple(Slot, dul:hasRegion, Obj),!.

get_slot_filler(Slot,Slot).