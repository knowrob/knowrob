
:- begin_tests('knowrob/computable').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/knowrob'),    [holds/3]).
:- use_module(library('knowrob/computable'), [rdfs_computable_triple/3, rdfs_computable_triple/4]).

:- owl_parser:owl_parse('package://knowrob_common/owl/comp_temporal.owl').
:- owl_parser:owl_parse('package://knowrob_common/owl/test_comp_temporal.owl').

:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

test(comp_during1) :-
  rdfs_computable_triple(ease:during, test_comp_temporal:'Short1', test_comp_temporal:'Long').

test(comp_during2) :-
  rdfs_computable_triple(ease:during, test_comp_temporal:'Short1', test_comp_temporal:'Long', _{}).

test(comp_during3) :-
  holds(test_comp_temporal:'Short1', ease:during, test_comp_temporal:'Long').

test(comp_simultaneous) :-
  holds(test_comp_temporal:'Long',ease:simultaneous,test_comp_temporal:'Long'),
  \+ holds(test_comp_temporal:'Long',ease:simultaneous,test_comp_temporal:'Short1').

test(comp_before) :-
  holds(test_comp_temporal:'Short1',ease:before,test_comp_temporal:'Short3'),
  \+ holds(test_comp_temporal:'Short3',ease:before,test_comp_temporal:'Short1'),
  \+ holds(test_comp_temporal:'Short1',ease:before,test_comp_temporal:'Short2'),
  \+ holds(test_comp_temporal:'Short2',ease:before,test_comp_temporal:'Short1').

test(comp_after) :-
  holds(test_comp_temporal:'Short3',ease:after,test_comp_temporal:'Short1'),
  \+ holds(test_comp_temporal:'Short1',ease:after,test_comp_temporal:'Short3'),
  \+ holds(test_comp_temporal:'Short1',ease:after,test_comp_temporal:'Short2'),
  \+ holds(test_comp_temporal:'Short2',ease:after,test_comp_temporal:'Short1').

test(comp_overlaps) :-
  holds(test_comp_temporal:'Long',ease:overlappedOn,test_comp_temporal:'Short4'),
  \+ holds(test_comp_temporal:'Short4',ease:overlappedOn,test_comp_temporal:'Long').

test(comp_overlappedBy) :-
  holds(test_comp_temporal:'Short4',ease:overlappedBy,test_comp_temporal:'Long'),
  \+ holds(test_comp_temporal:'Long',ease:overlappedBy,test_comp_temporal:'Short4').

test(comp_meets) :-
  holds(test_comp_temporal:'Short1',ease:meets,test_comp_temporal:'Short2'),
  \+ holds(test_comp_temporal:'Short1',ease:meets,test_comp_temporal:'Short3').

test(comp_metBy) :- 
  holds(test_comp_temporal:'Short2',ease:metBy,test_comp_temporal:'Short1'),
  \+ holds(test_comp_temporal:'Short3',ease:metBy,test_comp_temporal:'Short1').

test(comp_during) :-
  holds(test_comp_temporal:'Short2',ease:during,test_comp_temporal:'Long'),
  \+ holds(test_comp_temporal:'Short1',ease:during,test_comp_temporal:'Short2').

test(comp_starts) :- 
  holds(test_comp_temporal:'Short1',ease:starts,test_comp_temporal:'Long').

test(comp_startedBy) :- 
  holds(test_comp_temporal:'Long',ease:startedBy,test_comp_temporal:'Short1').

test(comp_finishes) :- 
  holds(test_comp_temporal:'Short3',ease:finishes,test_comp_temporal:'Long').

test(comp_finishedBy) :-
  holds(test_comp_temporal:'Long',ease:finishedBy,test_comp_temporal:'Short3'),
  \+ holds(test_comp_temporal:'Long',ease:finishedBy,test_comp_temporal:'Short4').

% TODO: test caching
% TODO: test temporal properties

:- end_tests('knowrob/computable').
