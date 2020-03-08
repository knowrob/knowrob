
:- module('knowrob/model/Region',
    [
      kb_is_region/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/lang/ask'), [ kb_type_of/2 ]).

:- rdf_meta kb_is_region(r).

%% kb_is_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Region'.
%
% @param Entity An entity IRI.
%
kb_is_region(Entity) :-
  kb_type_of(Entity,dul:'Region'),!.

