
:- module('knowrob/model/Reification',
    [
      kb_reification/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/lang/ask'),  [ kb_triple/3 ]).
:- use_module(library('knowrob/lang/tell'), [ kb_assert/3, kb_create/2 ]).

:- rdf_meta kb_reification(r,t).

%%
kb_reification(Resource,Reification) :-
  % FIXME: why does term expansion not work here? e.g. action_execution unit test
  kb_triple(Reification,'http://www.ease-crc.org/ont/EASE.owl#isReificationOf',Resource),!.

kb_reification(Resource,Reification) :-
  atom(Resource),
  kb_create('http://www.ease-crc.org/ont/EASE.owl#Reification',Reification),
  kb_assert(Reification,'http://www.ease-crc.org/ont/EASE.owl#isReificationOf',Resource).
