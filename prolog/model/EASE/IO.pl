
:- module(model_EASE_IO,
    [
      assignment_argument(r,r),
      assignment_value(r,r),
      assignment_set_argument(r,r),
      assignment_set_value(r,r),
      situation_assigns(r,r,r),
      situation_add_assigns(r,r,r)
    ]).
:- rdf_module.
/** <module> Interface predicates for EASE-IO model.

@author Daniel Beßler
@license BSD
*/

		 /*******************************
		 *	Assignments		*
		 *******************************/
% FIXME: Assignment defined in knowrob.owl

%% 
assignment_argument(Assignment,Argument) :-
  ask( Assignment dul:includesObject Argument ),
  ask( Argument rdf:type ease_io:'DigitalObject' ).

%% 
assignment_value(Assignment,Value) :-
  % TODO: better use more specific property
  ask( Assignment dul:isSettingFor Value ),
  assignment_argument(Assignment,X),
  Value \= X.

%% 
assignment_set_argument(Assignment,Argument) :-
  situation_add(Assignment,Argument).

%% 
assignment_set_value(Assignment,Value) :-
  situation_add(Assignment,Value).

%% situation_assigns(?Sit,?Argument,?Value) is nondet.
%
% Associates a situation to an argument assignment that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Argument An individual of type knowrob:'ProcedureArgument'.
% @param Value The RDF value of the argument.
%
situation_assigns(Sit,Argument,Value) :-
  ground(Argument) ->
  (situation_assigns_(Sit,Argument,Value),!);
  (situation_assigns_(Sit,Argument,Value)).

situation_assigns_(Sit,Argument,Value) :-
  ask( Sit        dul:hasPart Assignment ),
  ask( Assignment rdf:type    knowrob:'Assignment' ),
  assignment_argument(Assignment,Argument),
  assignment_value(Assignment,Value).

%% situation_add_assigns(?Sit,?Argument,?Value) is nondet.
%
% Associates a situation to an argument assignment that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Argument An individual of type knowrob:'ProcedureArgument'.
% @param Value The RDF value of the argument.
%
situation_add_assigns(Sit,Argument,Value) :-
  new( knowrob:'Assignment', Assignment ),
  assignment_set_argument(Assignment,Argument),
  assignment_set_value(Assignment,Value),
  situation_add(Sit,Assignment).
