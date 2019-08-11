/*
  Copyright (C) 2011 Moritz Tenorth
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

:- module(knowrob_actions,
    [
      task_role/3,
      task_parameter/3,
      action_task/2,
      action_set_task/2
    ]).
/** <module> Methods for reasoning about action descriptions

@author Moritz Tenorth
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).


:- rdf_meta
      task_role(r,r,r),
      task_parameter(r,r,r),
      action_task(r,r).

%%
task_role(Tsk,Role,ObjectType) :-
  rdfs_individual_of(Tsk,dul:'Task'),!,
  rdfs_type_of(Tsk,TskType),
  task_role(TskType,Role,ObjectType).

task_role(Tsk,Role,ObjectType) :-
  owl_subclass_of(Tsk,Restr),
  rdf_has(Restr,owl:onProperty,dul:isTaskOf),
  owl_restriction_object_domain(Restr,RoleDescr),
  once((
    (\+ ground(Role) ; owl_subclass_of(RoleDescr,Role)),
    owl_property_range_on_class(RoleDescr,dul:classifies,ObjectType),
    rdfs_individual_of(ObjectType,owl:'Class')
  )).

%%
task_parameter(Tsk,Param,Type) :-
  rdfs_individual_of(Tsk,dul:'Task'),
  rdfs_type_of(Tsk,TskType),
  task_parameter(TskType,Param,Type).

task_parameter(Tsk,Param,Type) :-
  owl_subclass_of(Tsk,Restr),
  rdf_has(Restr,owl:onProperty,dul:hasParameter),
  owl_restriction_object_domain(Restr,ParamDescr),
  once((
    (\+ ground(Param) ; owl_subclass_of(ParamDescr,Param)),
    owl_property_range_on_class(ParamDescr,dul:classifies,Type),
    rdfs_individual_of(Type,owl:'Class')
  )).

%%
action_task(Act,Tsk) :-
  rdfs_individual_of(Act,dul:'Action'),
  rdf_has(Act,dul:isClassifiedBy,Tsk_individual),
  rdfs_type_of(Tsk_individual,dul:'Task',Tsk).

%%
action_set_task(Act,Tsk) :-
  rdfs_individual_of(Act,dul:'Action'),
  rdfs_individual_of(Tsk,dul:'Task'),
  rdf_assert(Act,dul:isClassifiedBy,Tsk).
