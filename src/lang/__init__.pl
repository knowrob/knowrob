
:- use_module(library('semweb/rdf_db'), [ rdf_meta/1 ]).

:- rdf_meta(triple(t,t,t)).

:- use_module(messages).
:- use_module(subgraph).
:- use_module(scope).
:- use_module(db).
:- use_module(rdf_tests).

:- use_module(compiler).
:- use_module(query).
:- use_directory(mongolog).
:- use_directory(terms).
:- use_module(designator).
%:- use_module(computable).
