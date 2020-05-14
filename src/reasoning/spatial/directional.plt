
:- begin_tests('reasoning/spatial/directional').

%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('semweb/owl')).
%:- use_module(library('semweb/owl_parser')).
%:- use_module(library('knowrob/lang/holds')).
%:- use_module(library('knowrob/reasoning/spatial/simple')).

%:- owl_parser:owl_parse('package://knowrob/owl/test/test_comp_spatial.owl').

%:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(test_sp, 'http://knowrob.org/kb/test_comp_spatial.owl#', [keep(true)]).

%test(inCenterOf0) :-
  %get_time(Time),
  %comp_inCenterOf(test_sp:'cup3', test_sp:'cupboard1',Time),!.
%test(inCenterOf1) :-
  %holds(test_sp:'cup3', knowrob:isInCenterOf, test_sp:'cupboard1'),!.
%test(inCenterOf2) :-
  %holds(test_sp:'cup3', knowrob:isInCenterOf, A),
  %rdf_equal(A, test_sp:'cupboard1'),!.
%test(inCenterOf3) :-
  %holds(A, knowrob:isInCenterOf, test_sp:'cupboard1'),
  %rdf_equal(A, test_sp:'cup3'),!.


%test(in_ContGeneric1) :-
  %holds(test_sp:'cup3', knowrob:'isInsideOf', A),
  %rdf_equal(A, test_sp:'cupboard1'),!.
%test(in_ContGeneric2) :-
  %holds(A, knowrob:'isInsideOf', test_sp:'cupboard1'),
  %rdf_equal(A, test_sp:'cup3'),!.
%test(in_ContGeneric3) :-
  %holds(test_sp:'cup3', knowrob:'isInsideOf', test_sp:'cupboard1'),!.


%test(inFrontOf1) :-
  %holds(test_sp:'cup1', knowrob:'isInFrontOf', A),
  %rdf_equal(A, test_sp:'cup2'),!.
%test(inFrontOf2) :-
  %holds(A, knowrob:'isInFrontOf', test_sp:'cup2'),
  %rdf_equal(A, test_sp:'cup1'),!.
%test(inFrontOf3) :-
  %holds(test_sp:'cup1', knowrob:'isInFrontOf', test_sp:'cup2'),!.


%test(on_physical1) :-
  %holds(test_sp:'cup1', knowrob:'isOntopOf', A),
  %rdf_equal(A, test_sp:'cupboard1'),!.
%test(on_physical2) :-
  %holds(A, knowrob:'isOntopOf', test_sp:'cupboard1'),
  %rdf_equal(A, test_sp:'cup1'),!.
%test(on_physical3) :-
  %holds(test_sp:'cup1', knowrob:'isOntopOf', test_sp:'cupboard1'),!.

  
%test(aboveOf1) :-
  %holds(test_sp:'cup1', knowrob:'isAboveOf', A),
  %rdf_equal(A, test_sp:'cupboard1'),!.
%test(aboveOf2) :-
  %holds(A, knowrob:'isAboveOf', test_sp:'cupboard1'),
  %rdf_equal(A, test_sp:'cup1'),!.
%test(aboveOf3) :-
  %holds(test_sp:'cup1', knowrob:'isAboveOf', test_sp:'cupboard1'),!.

  
%% test(belowOf1) :-
%%   holds(test_sp:'cupboard1', knowrob:'isBelowOf', A),
%%   rdf_equal(A, test_sp:'cup1'),!.
%test(belowOf2) :-
  %holds(A, knowrob:'isBelowOf', test_sp:'cup1'),
  %rdf_equal(A, test_sp:'cupboard1'),!.
%test(belowOf3) :-
  %holds(test_sp:'cupboard1', knowrob:'isBelowOf', test_sp:'cup1'),!.

  
%test(toTheLeftOf1) :-
  %holds(test_sp:'cup2', knowrob:'isLeftOf', A),
  %rdf_equal(A, test_sp:'cup1'),!.
%test(toTheLeftOf2) :-
  %holds(A, knowrob:'isLeftOf', test_sp:'cup1'),
  %rdf_equal(A, test_sp:'cup2'),!.
%test(toTheLeftOf3) :-
  %holds(test_sp:'cup2', knowrob:'isLeftOf', test_sp:'cup1'),!.

  
%test(toTheRightOf1) :-
  %holds(test_sp:'cup1', knowrob:'isRightOf', A),
  %rdf_equal(A, test_sp:'cup2'),!.
%test(toTheRightOf2) :-
  %holds(A, knowrob:'isRightOf', test_sp:'cup2'),
  %rdf_equal(A, test_sp:'cup1'),!.
%test(toTheRightOf3) :-
  %holds(test_sp:'cup1', knowrob:'isRightOf', test_sp:'cup2'),!.

  
%test(toTheSideOf1) :-
  %holds(test_sp:'cup1', knowrob:'isNextTo', A),
  %rdf_equal(A, test_sp:'cup2'),!.
%test(toTheSideOf2) :-
  %holds(A, knowrob:'isNextTo', test_sp:'cup2'),
  %rdf_equal(A, test_sp:'cup1'),!.
%test(toTheSideOf3) :-
  %holds(test_sp:'cup1', knowrob:'isNextTo', test_sp:'cup2'),!.

:- end_tests('reasoning/spatial/directional').

