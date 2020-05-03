:- module(lang_export,
    [ remember/1,
      memorize/1
    ]).
/** <module> ...

@author Daniel Beßler
@license BSD
*/

:- use_module(library('db/tripledb'),
    [ tripledb_import/1,
      tripledb_export/1
    ]).
:- use_module(library('db/obda'),
    [ obda_import/1,
      obda_export/1
    ]).
:- use_module(library('comm/notify'),
    [ notify/1 ]).

%%
%
remember( dir(Directory) ) :-
  tripledb_import(Directory),
  obda_import(Directory),
  % TODO: only notify about individuals with new rdf:type
  %              assertion
  forall( is_individual(I),
          notify(individual(I)) ).

%%
%
memorize( dir(Directory) ) :-
  tripledb_export(Directory),
  obda_export(Directory).
