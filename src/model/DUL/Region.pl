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
      has_data_value(r,?)
    ]).
/** <module> DUL notion of Region.

In DUL, Region is defined as:
  "Any region in a dimensional space (a dimensional space is a maximal Region), which can be used as a value for a quality of an Entity."

@author Daniel Beßler
@license BSD
*/

:- use_module(library('lang/scopes/temporal'),
    [ time_interval_data/3
    ]).
:- use_module(library('model/RDFS'),
    [ has_type/2
    ]).
:- use_module('./Event.pl',
    [ is_event/1
    ]).
:- use_module('./Situation.pl',
    [ is_situation/1
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2
    ]).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

%% is_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Region'.
%
% @param Entity An entity IRI.
%
is_region(Entity) ?+>
  has_type(Entity, dul:'Region').

%% is_parameter(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Parameter'.
%
% @param Entity An entity IRI.
%
is_parameter(Entity) ?+>
  has_type(Entity, dul:'Parameter').

%% is_space_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SpaceRegion'.
%
% @param Entity An entity IRI.
%
is_space_region(Entity) ?+>
  has_type(Entity, dul:'SpaceRegion').

%% is_amount(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Amount'.
%
% @param Entity An entity IRI.
%
is_amount(Entity) ?+>
  has_type(Entity, dul:'Amount').

%% is_physical_attribute(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'PhysicalAttribute'.
%
% @param Entity An entity IRI.
%
is_physical_attribute(Entity) ?+>
  has_type(Entity, dul:'PhysicalAttribute').

%% is_social_attribute(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SocialAttribute'.
%
% @param Entity An entity IRI.
%
is_social_attribute(Entity) ?+>
  has_type(Entity, dul:'SocialAttribute').

%% is_time_interval(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'TimeInterval'.
%
% @param Entity An entity IRI.
%
is_time_interval(Entity) ?+>
  has_type(Entity, dul:'TimeInterval').

%%
%
has_region(Entity,Region) ?+>
  holds(Entity, dul:hasRegion, Region).

%%
%
has_parameter(Entity,Param) ?+>
  holds(Entity,dul:hasParameter,Param).

%%
%
has_parameter(Entity,Param,ParamType) ?>
  holds(Entity,dul:hasParameter,Param),
  has_object_type(Param,ParamType).

%%
%
has_parameter_range(Entity,Param,Range) ?>
  holds(Entity,dul:hasParameter,Param),
  holds(Param,dul:classifies,only(Range)).

%%
%
has_assignment(Param,Region) ?+>
  holds(Param,dul:classifies,Region).

%%
%
has_data_value(Entity,DataValue) ?+>
  holds(Entity,dul:hasDataValue,DataValue).
