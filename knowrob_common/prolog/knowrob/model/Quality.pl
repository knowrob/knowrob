
:- module('knowrob/model/Quality',
    [
      kb_is_quality/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/lang/ask'), [ kb_type_of/2 ]).

:- rdf_meta kb_is_quality(r).

%% kb_is_quality(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Quality'.
%
% @param Entity An entity IRI.
%
kb_is_quality(Entity) :-
  kb_type_of(Entity,dul:'Quality'),!.
