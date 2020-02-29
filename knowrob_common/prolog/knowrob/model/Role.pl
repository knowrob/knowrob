
:- module(knowrob_model_Role,
    [
      role_create/3
    ]).
/** <module> Interface to RDF model of roles.

*Role* is defined as a *Concept* that classifies an *Object*.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      role_create(r,r,+).

%%
role_create(Type,Individual,Graph) :-
  kb_create(Type,Individual,_{graph:Graph}).
