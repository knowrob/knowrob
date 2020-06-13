:- module(model_DUL_Region,
    [ is_region(r),
      is_parameter(r),
      is_time_interval(r),
      is_space_region(r),
      is_amount(r),
      is_physical_attribute(r),
      is_social_attribute(r),
      has_region(r,r),
      has_parameter(r,r),
      has_parameter(r,r,r),
      has_parameter_range(r,r,r),
      has_assignment(r,r),
      has_data_value(r,?),
      has_time_interval(r,r)
    ]).
/** <module> DUL notion of Region.

In DUL, Region is defined as:
  "Any region in a dimensional space (a dimensional space is a maximal Region), which can be used as a value for a quality of an Entity."

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).
:- use_module('./Event.pl',
    [ is_event/1 ]).
:- use_module('./Situation.pl',
    [ is_situation/1 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2, tripledb_ask/3 ]).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

%% is_region(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Region'.
%
% @param Entity An entity IRI.
%
is_region(Entity) ?+>
  has_type(Entity, dul:'Region').

%% is_parameter(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Parameter'.
%
% @param Entity An entity IRI.
%
is_parameter(Entity) ?+>
  has_type(Entity, dul:'Parameter').

%% is_space_region(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'SpaceRegion'.
%
% @param Entity An entity IRI.
%
is_space_region(Entity) ?+>
  has_type(Entity, dul:'SpaceRegion').

%% is_amount(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Amount'.
%
% @param Entity An entity IRI.
%
is_amount(Entity) ?+>
  has_type(Entity, dul:'Amount').

%% is_physical_attribute(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'PhysicalAttribute'.
%
% @param Entity An entity IRI.
%
is_physical_attribute(Entity) ?+>
  has_type(Entity, dul:'PhysicalAttribute').

%% is_social_attribute(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'SocialAttribute'.
%
% @param Entity An entity IRI.
%
is_social_attribute(Entity) ?+>
  has_type(Entity, dul:'SocialAttribute').

%% is_time_interval(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'TimeInterval'.
%
% @param Entity An entity IRI.
%
is_time_interval(Entity) ?+>
  has_type(Entity, dul:'TimeInterval').

%% has_region(?Entity,?Region) is nondet.
%
% A relation between entities and regions, e.g.
% - 'the number of wheels of that truck is 12',
% - 'the time of the experiment is August 9th, 2004',
% - 'the whale has been localized at 34 degrees E, 20 degrees S'.
%
% @param Entity An entity IRI.
% @param Region An region IRI.
%
has_region(Entity,Region) ?+>
  holds(Entity, dul:hasRegion, Region).

%% has_parameter(?Entity,?Param) is nondet.
%
% A Concept can have a Parameter that constrains the attributes
% that a classified Entity can have in a certain Situation,
% e.g. a WheelDriver Role definedIn the ItalianTrafficLaw has
% a MinimumAge parameter on the Amount 16.
%
% @param Entity An entity IRI.
% @param Param An parameter IRI.
%
has_parameter(Entity,Param) ?+>
  holds(Entity,dul:hasParameter,Param).

%% has_parameter(?Entity,?Param,?ParamType) is nondet.
%
% Same as has_parameter/2 but in addition unifies the
% type of the parameter with the third argument.
%
% @param Entity entity IRI.
% @param Param parameter IRI.
% @param Param parameter type IRI.
%
has_parameter(Entity,Param,ParamType) ?>
  holds(Entity,dul:hasParameter,Param),
  has_object_type(Param,ParamType).

%% has_parameter_range(?Entity,?Param,?Range) is nondet.
%
% Same as has_parameter/2 but in addition unifies the
% range of the parameter with the third argument.
%
% @param Entity entity IRI.
% @param Param parameter IRI.
% @param Range parameter range IRI.
%
has_parameter_range(Entity,Param,Range) ?>
  holds(Entity,dul:hasParameter,Param),
  holds(Param,dul:classifies,only(Range)).

%% has_assignment(?Param,?Region) is nondet.
%
% Associates a parameter to an assignment.
%
% @param Param parameter IRI.
% @param Region region IRI.
%
has_assignment(Param,Region) ?+>
  holds(Param,dul:classifies,Region).

%% has_data_value(?Entity,?DataValue) is nondet.
%
% A datatype property that encodes values from a datatype for an Entity. 
%
% @param Entity entity IRI.
% @param DataValue typed data value.
%
has_data_value(Entity,DataValue) ?+>
  holds(Entity,dul:hasDataValue,DataValue).

%% has_time_interval(+Entity,?Interval) is semidet. 
%
%
has_time_interval(Entity,TimeInterval) ?+>
  triple(Entity,dul:hasTimeInterval,TimeInterval).
