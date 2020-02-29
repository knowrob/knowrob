
:- module(knowrob_model_Task,
    [
      task_role/3,
      task_parameter/3,
      task_role_range/3,
      task_parameter_range/3
    ]).
/** <module> Interface to RDF model of tasks.

*Task* is defined as an *EventType* that *classifies* an *Action* to be executed. 
For example, reaching a destination is a task that can be executed by performing certain actions, e.g. driving a car, buying a train ticket, etc. 
The actions to execute a task can also be organized according to a *Plan* that is not the same as the one that defines the task (if any). 
For example, reaching a destination could be defined by a plan to get on holidays, while the plan to execute the task can consist of putting some travels into a sequence.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      task_role(r,r,r),
      task_parameter(r,r,r),
      task_role_range(r,r,r),
      task_parameter_range(r,r,r).

%% task_role(?Tsk,?Role,?RoleType) is semidet.
%
% Relates a task to roles of objects related to the tasks.
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
% @param RoleType A sub-class of dul:'Role'.
%
task_role(Tsk,Role,RoleType) :-
  kb_triple(Tsk,dul:isTaskOf,Role),
  kb_type_of(Role,RoleType).

task_parameter(Tsk,Param,ParamType) :-
  kb_triple(Tsk,dul:hasParameter,Param),
  kb_type_of(Param,ParamType).

%%
task_role_range(Tsk,Role,Obj) :-
  property_cardinality(Tsk,dul:isTaskOf,RoleDescr,CR,_), CR>0,
  property_range(RoleDescr,dul:classifies,ObjDescr),
  once((
    %%
    owl_subclass_of(RoleDescr,Role),
    rdfs_subclass_of(Role,dul:'Role'),
    %%
    owl_subclass_of(ObjDescr,Obj),
    rdfs_subclass_of(Obj,dul:'Object')
  )).

%%
task_parameter_range(Tsk,Parameter,Region) :-
  property_cardinality(Tsk,dul:hasParameter,ParamDescr,CR,_), CR>0,
  property_range(ParamDescr,dul:classifies,RegionDescr),
  once((
    %%
    owl_subclass_of(ParamDescr,Parameter),
    rdfs_subclass_of(Parameter,dul:'Parameter'),
    %%
    owl_subclass_of(RegionDescr,Region),
    rdfs_subclass_of(Region,dul:'Region')
  )).
