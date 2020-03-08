
:- module('knowrob/model/Role',
    [
      role_create/3
    ]).
/** <module> Interface to RDF model of roles.

*Role* is defined as a *Concept* that classifies an *Object*.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/lang/tell'), [ kb_create/3 ]).

:- rdf_meta
      role_create(r,r,+).

%%
role_create(Type,Individual,Graph) :-
  kb_create(Type,Individual,_{graph:Graph}).
