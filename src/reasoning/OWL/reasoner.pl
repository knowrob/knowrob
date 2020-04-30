:- module(owl_reasoner,
    [ implements('../ireasoner.pl')
    ]).

:- use_module(library('plowl/class')).
:- use_module(library('plowl/individual')).
:- use_module(library('plowl/property')).

%%
% @implements ireasoner
%
can_answer(holds(_,_,_)).
can_answer(instance_of(_,_)).
can_answer(subclass_of(_,_)).
can_answer(subproperty_of(_,_)).

%%
% @implements ireasoner
%
% TODO: generate more specific facts if possible
%
infer( holds(S,P,O),
       holds(S,P,O),
       QScope,FScope) :-
  { owl_has(S,P,O,QScope->FScope) }.

infer( is_instance_of(S,O),
       is_instance_of(S,O),
       QScope,FScope) :-
  { owl_individual_of(S,O,QScope->FScope) }.

infer( is_subclass_of(S,O),
       is_subclass_of(S,O),
       _QScope,_{}) :-
  { owl_subclass_of(S,O) }.

infer( is_subproperty_of(S,O),
       is_subproperty_of(S,O),
       _QScope,_{}) :-
  { owl_subproperty_of(S,O) }.
