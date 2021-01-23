:- module(owl_reasoner,
    [ implements('../ireasoner.pl')
    ]).

:- use_module(library('lang/scope'), [ universal_scope/1 ]).

:- use_module('./plowl/class.pl',      [ owl_subclass_of/2 ]).
:- use_module('./plowl/individual.pl', [ owl_individual_of/3 ]).
:- use_module('./plowl/property.pl',   [ owl_subproperty_of/2,
                                         owl_has/4 ]).

:- register_reasoner(owl_reasoner).

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
  owl_has(S,P,O,QScope->FScope).

infer( instance_of(S,O),
       instance_of(S,O),
       QScope,FScope) :-
  owl_individual_of(S,O,QScope->FScope).

infer(subclass_of(S,O), _, _, _) :-
  ground([S,O]), S=O, !,
  fail.

infer( subclass_of(S,O),
       subclass_of(S,O),
       _QScope,
       FScope) :-
  universal_scope(FScope),
  owl_subclass_of(S,O),
  S\=O.

infer(subproperty_of(S,O), _, _, _) :-
  ground([S,O]), S=O, !,
  fail.

infer( subproperty_of(S,O),
       subproperty_of(S,O),
       _QScope,
       FScope) :-
  universal_scope(FScope),
  owl_subproperty_of(S,O).
