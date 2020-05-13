:- module(model_EASE_IO,
    [ is_computational_agent(r),
      is_digital_object(r)
    ]).
/** <module> ....

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2
    ]).

:- tripledb_load('http://www.ease-crc.org/ont/EASE-IO.owl',
    [ graph(tbox),
      namespace(ease_io)
    ]).

%% is_computational_agent(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'ManipulationAction'.
%
% @param Entity An entity IRI.
%
is_computational_agent(Entity) ?+>
  has_type(Entity, ease_io:'ComputationalAgent').

%% is_digital_object(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'ManipulationAction'.
%
% @param Entity An entity IRI.
%
is_digital_object(Entity) ?+>
  has_type(Entity, ease_io:'DigitalObject').
