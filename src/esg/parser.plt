
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rostest')).
:- use_module('parser').

:- begin_rdf_tests(
		'activity_parser',
		'package://knowrob/owl/test/parser-test.owl',
		[ namespace('http://knowrob.org/kb/parser-test.owl#')
		]).

:- dynamic test_parser/1.
:- rdf_meta test_parser_run(t,t),
            test_composer_run(t,t),
            test_parser_run_asynch(t,t).

test('parser_assert') :-
  parser_create(Parser, [
      test:'Grasping_WF0',
      test:'Lifting_WF0',
      test:'PickingUp_WF0',
      test:'Placing_WF0',
      test:'PickPlace_WF0'
  ]),
  assertz(test_parser(Parser)).

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

%tests('action_parser(Placing)', [nondet]) :-
%  test_parser_run([
%    tok(0.0, -(tests:'Touching'),        [tests:'TestHand',tests:'TestObject']),
%    tok(0.9, -(tests:'Supporting'),      [tests:'TestTable',tests:'TestObject']),
%    tok(1.0, -(tests:'ReleaseMotion'),   [tests:'TestHand']),
%    tok(3.0, +(tests:'Touching'),        [tests:'TestHand',tests:'TestObject']),
%    tok(4.0, +(tests:'ReleaseMotion'),   [tests:'TestHand'])
%    ],
%    [action(_,tests:'Placing',_)]).
%
%tests('action_parser(asynch)', [nondet]) :-
%  test_parser_run_asynch([
%    tok(0.0, -(tests:'Touching'),        [tests:'TestHand',tests:'TestObject']),
%    tok(0.9, -(tests:'Supporting'),      [tests:'TestTable',tests:'TestObject']),
%    tok(1.0, -(tests:'ReleaseMotion'),   [tests:'TestHand']),
%    tok(3.0, +(tests:'Touching'),        [tests:'TestHand',tests:'TestObject']),
%    tok(4.0, +(tests:'ReleaseMotion'),   [tests:'TestHand'])
%    ],
%    [action(_,tests:'Placing',_)]).
%
%tests('action_parser(PickingUp)', [nondet]) :-
%  test_parser_run([
%    tok(0.0,-(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject']),
%    tok(1.0,-(tests:'GraspMotion'),    [tests:'TestHand']),
%    tok(3.0,-(tests:'Touching'),       [tests:'TestHand',tests:'TestObject']),
%    tok(4.0,+(tests:'GraspMotion'),    [tests:'TestHand']),
%    tok(5.0,+(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject'])
%    ],
%    [action(_,tests:'PickingUp',_)]).
%
%tests('action_parser(not PickingUp)', [fail]) :-
%  test_parser_run([
%    tok(0.0, -(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject']),
%    tok(0.5, +(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject']),
%    tok(1.0, -(tests:'GraspMotion'),    [tests:'TestHand']),
%    tok(3.0, -(tests:'Touching'),       [tests:'TestHand',tests:'TestObject']),
%    tok(4.0, +(tests:'GraspMotion'),    [tests:'TestHand'])
%    ],
%    [action(_,tests:'PickingUp',_)]).
%
%tests('action_parser(PickPlace)', [nondet]) :-
%  test_parser_run([
%    tok(0.0, -(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject']),
%    tok(1.0, -(tests:'GraspMotion'),    [tests:'TestHand']),
%    tok(3.0, -(tests:'Touching'),       [tests:'TestHand',tests:'TestObject']),
%    tok(4.0, +(tests:'GraspMotion'),    [tests:'TestHand']),
%    tok(5.0, +(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject']),
%    tok(8.0, -(tests:'Supporting'),     [tests:'TestTable',tests:'TestObject']),
%    tok(9.0, -(tests:'ReleaseMotion'),  [tests:'TestHand']),
%    tok(9.5, +(tests:'Touching'),       [tests:'TestHand',tests:'TestObject']),
%    tok(10.0,+(tests:'ReleaseMotion'),  [tests:'TestHand'])
%    ],
%    [action(_,tests:'PickPlace',_)]).
%
%tests('action_parser(PickPlace2)', [nondet]) :-
%  test_parser_run([
%    %%%% first
%    tok(0.0, -(tests:'Supporting'),    [tests:'TestTable',tests:'TestObject']),
%    tok(1.0, -(tests:'GraspMotion'),   [tests:'TestHand']),
%    tok(3.0, -(tests:'Touching'),      [tests:'TestHand',tests:'TestObject']),
%    tok(4.0, +(tests:'GraspMotion'),   [tests:'TestHand']),
%    tok(5.0, +(tests:'Supporting'),    [tests:'TestTable',tests:'TestObject']),
%    tok(8.0, -(tests:'Supporting'),    [tests:'TestTable',tests:'TestObject']),
%    tok(9.0, -(tests:'ReleaseMotion'), [tests:'TestHand']),
%    tok(9.5, +(tests:'Touching'),      [tests:'TestHand',tests:'TestObject']),
%    tok(10.0,+(tests:'ReleaseMotion'), [tests:'TestHand']),
%    %%%% second
%    tok(21.0, -(tests:'GraspMotion'),   [tests:'TestHand']),
%    tok(23.0, -(tests:'Touching'),      [tests:'TestHand',tests:'TestObject']),
%    tok(24.0, +(tests:'GraspMotion'),   [tests:'TestHand']),
%    tok(25.0, +(tests:'Supporting'),    [tests:'TestTable',tests:'TestObject']),
%    tok(28.0, -(tests:'Supporting'),    [tests:'TestTable',tests:'TestObject']),
%    tok(29.0, -(tests:'ReleaseMotion'), [tests:'TestHand']),
%    tok(29.5, +(tests:'Touching'),      [tests:'TestHand',tests:'TestObject']),
%    tok(30.0, +(tests:'ReleaseMotion'), [tests:'TestHand'])
%    ],
%    [action(_,tests:'PickPlace',_),action(_,tests:'PickPlace',_)]
%  ).

test('parser_retract') :-
  test_parser(Parser),
  retractall(parser_grammar(Parser,_,_)).

:- end_tests('activity_parser').
