:- begin_tests('knowrob/action_parser').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/action_parser')).

:- owl_parser:owl_parse('package://knowrob_actions/owl/parser-test.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ptest, 'http://knowrob.org/kb/parser-test.owl#', [keep(true)]).

:- dynamic test_parser/1.
:- rdf_meta test_parser_run(t,t),
            test_composer_run(t,t),
            test_parser_run_asynch(t,t).

test('parser_assert') :-
  parser_create(Parser, [
      ptest:'Grasping_WF0',
      ptest:'Lifting_WF0',
      ptest:'PickingUp_WF0',
      ptest:'Placing_WF0',
      ptest:'PickPlace_WF0'
  ]),
  assertz(test_parser(Parser)).

test_parser_run(Tokens,Expected) :-
  test_parser(Parser),
  parser_run(Parser,Tokens,Actual),
  Actual = Expected.

test_parser_run_asynch(Tokens,Expected) :-
  test_parser(Parser),
  parser_start(Parser),
  forall(member(Tok,Tokens), parser_push_token(Parser,Tok)),
  parser_stop(Parser,Actual),
  Actual = Expected.

test('action_parser(Placing)', [nondet]) :-
  test_parser_run([
    tok(0.0, -(ptest:'Touching'),        [ptest:'TestHand',ptest:'TestObject']),
    tok(0.9, -(ptest:'Supporting'),      [ptest:'TestTable',ptest:'TestObject']),
    tok(1.0, -(ptest:'ReleaseMotion'),   [ptest:'TestHand']),
    tok(3.0, +(ptest:'Touching'),        [ptest:'TestHand',ptest:'TestObject']),
    tok(4.0, +(ptest:'ReleaseMotion'),   [ptest:'TestHand'])
    ],
    [action(_,ptest:'Placing',_)]).

test('action_parser(asynch)', [nondet]) :-
  test_parser_run_asynch([
    tok(0.0, -(ptest:'Touching'),        [ptest:'TestHand',ptest:'TestObject']),
    tok(0.9, -(ptest:'Supporting'),      [ptest:'TestTable',ptest:'TestObject']),
    tok(1.0, -(ptest:'ReleaseMotion'),   [ptest:'TestHand']),
    tok(3.0, +(ptest:'Touching'),        [ptest:'TestHand',ptest:'TestObject']),
    tok(4.0, +(ptest:'ReleaseMotion'),   [ptest:'TestHand'])
    ],
    [action(_,ptest:'Placing',_)]).

test('action_parser(PickingUp)', [nondet]) :-
  test_parser_run([
    tok(0.0,-(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject']),
    tok(1.0,-(ptest:'GraspMotion'),    [ptest:'TestHand']),
    tok(3.0,-(ptest:'Touching'),       [ptest:'TestHand',ptest:'TestObject']),
    tok(4.0,+(ptest:'GraspMotion'),    [ptest:'TestHand']),
    tok(5.0,+(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject'])
    ],
    [action(_,ptest:'PickingUp',_)]).

test('action_parser(not PickingUp)', [fail]) :-
  test_parser_run([
    tok(0.0, -(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject']),
    tok(0.5, +(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject']),
    tok(1.0, -(ptest:'GraspMotion'),    [ptest:'TestHand']),
    tok(3.0, -(ptest:'Touching'),       [ptest:'TestHand',ptest:'TestObject']),
    tok(4.0, +(ptest:'GraspMotion'),    [ptest:'TestHand'])
    ],
    [action(_,ptest:'PickingUp',_)]).

test('action_parser(PickPlace)', [nondet]) :-
  test_parser_run([
    tok(0.0, -(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject']),
    tok(1.0, -(ptest:'GraspMotion'),    [ptest:'TestHand']),
    tok(3.0, -(ptest:'Touching'),       [ptest:'TestHand',ptest:'TestObject']),
    tok(4.0, +(ptest:'GraspMotion'),    [ptest:'TestHand']),
    tok(5.0, +(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject']),
    tok(8.0, -(ptest:'Supporting'),     [ptest:'TestTable',ptest:'TestObject']),
    tok(9.0, -(ptest:'ReleaseMotion'),  [ptest:'TestHand']),
    tok(9.5, +(ptest:'Touching'),       [ptest:'TestHand',ptest:'TestObject']),
    tok(10.0,+(ptest:'ReleaseMotion'),  [ptest:'TestHand'])
    ],
    [action(_,ptest:'PickPlace',_)]).

test('action_parser(PickPlace2)', [nondet]) :-
  test_parser_run([
    %%%% first
    tok(0.0, -(ptest:'Supporting'),    [ptest:'TestTable',ptest:'TestObject']),
    tok(1.0, -(ptest:'GraspMotion'),   [ptest:'TestHand']),
    tok(3.0, -(ptest:'Touching'),      [ptest:'TestHand',ptest:'TestObject']),
    tok(4.0, +(ptest:'GraspMotion'),   [ptest:'TestHand']),
    tok(5.0, +(ptest:'Supporting'),    [ptest:'TestTable',ptest:'TestObject']),
    tok(8.0, -(ptest:'Supporting'),    [ptest:'TestTable',ptest:'TestObject']),
    tok(9.0, -(ptest:'ReleaseMotion'), [ptest:'TestHand']),
    tok(9.5, +(ptest:'Touching'),      [ptest:'TestHand',ptest:'TestObject']),
    tok(10.0,+(ptest:'ReleaseMotion'), [ptest:'TestHand']),
    %%%% second
    tok(21.0, -(ptest:'GraspMotion'),   [ptest:'TestHand']),
    tok(23.0, -(ptest:'Touching'),      [ptest:'TestHand',ptest:'TestObject']),
    tok(24.0, +(ptest:'GraspMotion'),   [ptest:'TestHand']),
    tok(25.0, +(ptest:'Supporting'),    [ptest:'TestTable',ptest:'TestObject']),
    tok(28.0, -(ptest:'Supporting'),    [ptest:'TestTable',ptest:'TestObject']),
    tok(29.0, -(ptest:'ReleaseMotion'), [ptest:'TestHand']),
    tok(29.5, +(ptest:'Touching'),      [ptest:'TestHand',ptest:'TestObject']),
    tok(30.0, +(ptest:'ReleaseMotion'), [ptest:'TestHand'])
    ],
    [action(_,ptest:'PickPlace',_),action(_,ptest:'PickPlace',_)]
  ).

test('parser_retract') :-
  test_parser(Parser),
  retractall(parser_grammar(Parser,_,_)).

:- end_tests('knowrob/action_parser').
