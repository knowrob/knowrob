:- use_module(library('rdf_test')).
:- begin_rdf_tests('activity_parser', 'owl/test/parser-test.owl').

:- use_module('parser.pl', [ parser_create/2 ]).
:- use_module(library('semweb/rdf_db')).

:- rdf_register_prefix(test, 'http://knowrob.org/kb/parser-test.owl#', [force(true)]).

:- dynamic test_parser/1.
:- rdf_meta test_parser_run(t,t),
            test_composer_run(t,t),
            test_parser_run_asynch(t,t).

test_parser_run(Tokens,Expected) :-
  test_parser(Parser),
  parser_run(Parser,Tokens,Actual),
  Actual=Expected.

test_parser_run_asynch(Tokens,Expected) :-
  test_parser(Parser),
  parser_start(Parser),
  forall(member(Tok,Tokens), parser_push_token(Parser,Tok)),
  parser_stop(Parser,Actual),
  Actual=Expected.

test('parser_assert') :-
  parser_create(Parser, [
      test:'Grasping_WF0',
      test:'Lifting_WF0',
      test:'PickingUp_WF0',
      test:'Placing_WF0',
      test:'PickPlace_WF0'
  ]),
  assertz(test_parser(Parser)).

test('action_parser(Placing)', [fixme('unknown reason')]) :-
  test_parser_run([
    tok(0.0, -(test:'Touching'),        [test:'TestHand',test:'TestObject']),
    tok(0.9, -(test:'Supporting'),      [test:'TestTable',test:'TestObject']),
    tok(1.0, -(test:'ReleaseMotion'),   [test:'TestHand']),
    tok(3.0, +(test:'Touching'),        [test:'TestHand',test:'TestObject']),
    tok(4.0, +(test:'ReleaseMotion'),   [test:'TestHand'])
    ],
    [action(_,test:'Placing',_)]).

test('action_parser(asynch)', [fixme('unknown reason')]) :-
  test_parser_run_asynch([
    tok(0.0, -(test:'Touching'),        [test:'TestHand',test:'TestObject']),
    tok(0.9, -(test:'Supporting'),      [test:'TestTable',test:'TestObject']),
    tok(1.0, -(test:'ReleaseMotion'),   [test:'TestHand']),
    tok(3.0, +(test:'Touching'),        [test:'TestHand',test:'TestObject']),
    tok(4.0, +(test:'ReleaseMotion'),   [test:'TestHand'])
    ],
    [action(_,test:'Placing',_)]).

test('action_parser(PickingUp)', [fixme('unknown reason')]) :-
  test_parser_run([
    tok(0.0,-(test:'Supporting'),     [test:'TestTable',test:'TestObject']),
    tok(1.0,-(test:'GraspMotion'),    [test:'TestHand']),
    tok(3.0,-(test:'Touching'),       [test:'TestHand',test:'TestObject']),
    tok(4.0,+(test:'GraspMotion'),    [test:'TestHand']),
    tok(5.0,+(test:'Supporting'),     [test:'TestTable',test:'TestObject'])
    ],
    [action(_,test:'PickingUp',_)]).

test('action_parser(not PickingUp)', [fixme('unknown reason')]) :-
  test_parser_run([
    tok(0.0, -(test:'Supporting'),     [test:'TestTable',test:'TestObject']),
    tok(0.5, +(test:'Supporting'),     [test:'TestTable',test:'TestObject']),
    tok(1.0, -(test:'GraspMotion'),    [test:'TestHand']),
    tok(3.0, -(test:'Touching'),       [test:'TestHand',test:'TestObject']),
    tok(4.0, +(test:'GraspMotion'),    [test:'TestHand'])
    ],
    [action(_,test:'PickingUp',_)]).

test('action_parser(PickPlace)', [fixme('unknown reason')]) :-
  test_parser_run([
    tok(0.0, -(test:'Supporting'),     [test:'TestTable',test:'TestObject']),
    tok(1.0, -(test:'GraspMotion'),    [test:'TestHand']),
    tok(3.0, -(test:'Touching'),       [test:'TestHand',test:'TestObject']),
    tok(4.0, +(test:'GraspMotion'),    [test:'TestHand']),
    tok(5.0, +(test:'Supporting'),     [test:'TestTable',test:'TestObject']),
    tok(8.0, -(test:'Supporting'),     [test:'TestTable',test:'TestObject']),
    tok(9.0, -(test:'ReleaseMotion'),  [test:'TestHand']),
    tok(9.5, +(test:'Touching'),       [test:'TestHand',test:'TestObject']),
    tok(10.0,+(test:'ReleaseMotion'),  [test:'TestHand'])
    ],
    [action(_,test:'PickPlace',_)]).

test('action_parser(PickPlace2)', [fixme('unknown reason')]) :-
  test_parser_run([
    %%%% first
    tok(0.0, -(test:'Supporting'),    [test:'TestTable',test:'TestObject']),
    tok(1.0, -(test:'GraspMotion'),   [test:'TestHand']),
    tok(3.0, -(test:'Touching'),      [test:'TestHand',test:'TestObject']),
    tok(4.0, +(test:'GraspMotion'),   [test:'TestHand']),
    tok(5.0, +(test:'Supporting'),    [test:'TestTable',test:'TestObject']),
    tok(8.0, -(test:'Supporting'),    [test:'TestTable',test:'TestObject']),
    tok(9.0, -(test:'ReleaseMotion'), [test:'TestHand']),
    tok(9.5, +(test:'Touching'),      [test:'TestHand',test:'TestObject']),
    tok(10.0,+(test:'ReleaseMotion'), [test:'TestHand']),
    %%%% second
    tok(21.0, -(test:'GraspMotion'),   [test:'TestHand']),
    tok(23.0, -(test:'Touching'),      [test:'TestHand',test:'TestObject']),
    tok(24.0, +(test:'GraspMotion'),   [test:'TestHand']),
    tok(25.0, +(test:'Supporting'),    [test:'TestTable',test:'TestObject']),
    tok(28.0, -(test:'Supporting'),    [test:'TestTable',test:'TestObject']),
    tok(29.0, -(test:'ReleaseMotion'), [test:'TestHand']),
    tok(29.5, +(test:'Touching'),      [test:'TestHand',test:'TestObject']),
    tok(30.0, +(test:'ReleaseMotion'), [test:'TestHand'])
    ],
    [action(_,test:'PickPlace',_),action(_,test:'PickPlace',_)]
  ).

test('parser_retract') :-
  test_parser(Parser),
  retractall(parser_grammar(Parser,_,_)).

:- end_tests('activity_parser').
