
:- module(model_DnS_Description,
    [
      is_description(r),
      is_goal(r),
      is_design(r),
      is_diagnosis(r),
      is_plan(r),
      is_norm(r),
      plan_has_goal(r,r)
    ]).
:- rdf_module.
/** <module> DnS notion of Description.

In DnS, Description is defined as:
  "A SocialObject that represents a conceptualization. It can be thought also as a 'descriptive context' that uses or defines concepts in order to create a view on a 'relational context' (cf. Situation) out of a set of data or observations. For example, a Plan is a Description of some actions to be executed by agents in a certain way, with certain parameters; a Diagnosis is a Description that provides an interpretation for a set of observed entities, etc."

@author Daniel Beßler
@license BSD
*/

%% is_description(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Description'.
%
% @param Entity An entity IRI.
%
is_description(Entity) :-
  ask( Entity rdf:type dul:'Description' ).

%% is_goal(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Goal'.
%
% @param Entity An entity IRI.
%
is_goal(Entity) :-
  ask( Entity rdf:type dul:'Goal' ).

%% is_design(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Design'.
%
% @param Entity An entity IRI.
%
is_design(Entity) :-
  ask( Entity rdf:type dul:'Design' ).

%% is_diagnosis(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Diagnosis'.
%
% @param Entity An entity IRI.
%
is_diagnosis(Entity) :-
  ask( Entity rdf:type dul:'Diagnosis' ).

%% is_plan(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Plan'.
%
% @param Entity An entity IRI.
%
is_plan(Entity) :-
  ask( Entity rdf:type dul:'Plan' ).

%% is_norm(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Norm'.
%
% @param Entity An entity IRI.
%
is_norm(Entity) :-
  ask( Entity rdf:type dul:'Norm' ).

%% plan_has_goal(?Plan,?Goal) is semidet.
%
% Relates a plan to its goal.
%
% @param Plan An individual of type dul:'Plan'.
% @param Goal An individual of type dul:'Description'.
%
plan_has_goal(Plan,Goal) :-
  ask( Plan dul:hasComponent Goal ),
  ask( Goal rdf:type         dul:'Goal' ).
