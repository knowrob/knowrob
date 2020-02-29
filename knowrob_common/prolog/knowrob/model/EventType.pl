
:- module(knowrob_model_EventType,
    [
      event_type_create/3
    ]).
/** <module> Interface to RDF model of event types.

*EventType* is defined as a *Concept* that *classifies* an *Event*. An event type describes how an *Event* should be interpreted, executed, expected, seen, etc., according to the *Description* that the *EventType* *isDefinedIn* (or used in).

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- use_module(library('knowrob/model/Parameter'), [ parameter_create/3 ]).
:- use_module(library('knowrob/model/Role'),      [ role_create/3 ]).

:- rdf_meta
      event_type_create(r,r,+).

%%
event_type_create(Type,Individual,Graph) :-
  kb_create(Type,Individual,_{graph:Graph}),
  %%
  forall(
    ( property_cardinality(Individual,dul:isTaskOf,RoleDescr,Min0,_),
      between(1,Min0,_)
    ),
    ( role_create(RoleDescr,Role,Graph),
      kb_assert(Individual,dul:isTaskOf,Role)
    )
  ),
  %%
  forall(
    ( property_cardinality(Individual,dul:hasParameter,ParamDescr,Min1,_),
      between(1,Min1,_)
    ),
    ( parameter_create(ParamDescr,Param,Graph),
      kb_assert(Individual,dul:hasParameter,Param)
    )
  ).
