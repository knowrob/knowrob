:- module(model_EASE_IO,
    [ is_computational_agent(r),
      is_digital_object(r)
    ]).
/** <module> ....

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).

%% is_computational_agent(?Entity) is semidet.
%
% True iff Entity is an instance of soma:'ComputationalAgent'.
%
% @param Entity An entity IRI.
%
is_computational_agent(Entity) ?+>
  has_type(Entity, soma:'ComputationalAgent').

%% is_digital_object(?Entity) is semidet.
%
% True iff Entity is an instance of soma:'DigitalObject'.
%
% @param Entity An entity IRI.
%
is_digital_object(Entity) ?+>
  has_type(Entity, soma:'DigitalObject').
