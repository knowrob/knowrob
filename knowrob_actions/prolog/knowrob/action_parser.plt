:- begin_tests('knowrob/action_parser').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/action_parser')).

:- owl_parser:owl_parse('package://knowrob_actions/owl/parser-test.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_proc, 'http://www.ease-crc.org/ont/EASE-PROC.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ptest, 'http://knowrob.org/kb/parser-test.owl#', [keep(true)]).

:- dynamic test_parser/1.
:- rdf_meta test_detect_activity(t,t).

test('parser_assert') :-
  parser_create(Parser, [
      ptest:'Grasping_WF0',
      ptest:'Lifting_WF0',
      ptest:'PickingUp_WF0',
      ptest:'Placing_WF0',
      ptest:'PickPlace_WF0'
  ]),
  assertz(test_parser(Parser)),
  rdf_assert('TestTable', rdf:type, dul:'PhysicalArtifact'),
  rdf_assert('TestObject', rdf:type, dul:'PhysicalArtifact'),
  rdf_assert('TestHand', rdf:type, ease:'PhysicalEffector').

test_detect_activity(Tokens,Expected) :-
  %test_parser(Parser),
  detect_activity2(Tokens,Actual),
  ( Actual = Expected -> true ; (
    print_term(
       '!='(Expected,Actual),
       [indent_arguments(true)]
    ),
    fail
  )).

test('detect_activity(Placing)', [nondet]) :-
  test_detect_activity([
    tok(0.0,c, -(ptest:'Touching'),        ['TestHand','TestObject']),
    tok(0.9,a, -(ptest:'Supporting'),      ['TestTable','TestObject']),
    tok(1.0,b, -(ptest:'ReleaseMotion'),   ['TestHand']),
    tok(3.0,c, +(ptest:'Touching'),        ['TestHand','TestObject']),
    tok(4.0,b, +(ptest:'ReleaseMotion'),   ['TestHand'])
    ],
    [action(_,ptest:'Placing',_,_)]).

test('detect_activity(PickingUp)', [nondet]) :-
  test_detect_activity([
    tok(0.0,a,-(ptest:'Supporting'),     ['TestTable','TestObject']),
    tok(1.0,b,-(ptest:'GraspMotion'),    ['TestHand']),
    tok(3.0,c,-(ptest:'Touching'),       ['TestHand','TestObject']),
    tok(4.0,d,+(ptest:'GraspMotion'),    ['TestHand']),
    tok(5.0,f,+(ptest:'Supporting'),     ['TestTable','TestObject'])
    ],
    [action(_,ptest:'PickingUp',_,_)]).

test('detect_activity(not PickingUp)', [fail]) :-
  detect_activity2([
    tok(0.0,a, -(ptest:'Supporting'),     ['TestTable','TestObject']),
    tok(0.5,a, +(ptest:'Supporting'),     ['TestTable','TestObject']),
    tok(1.0,b, -(ptest:'GraspMotion'),    ['TestHand']),
    tok(3.0,c, -(ptest:'Touching'),       ['TestHand','TestObject']),
    tok(4.0,d, +(ptest:'GraspMotion'),    ['TestHand'])
    ],
    [action(_,ptest:'PickingUp',_,_)]).

test('detect_activity(PickPlace1)', [nondet]) :-
  detect_activity2([
    tok(0.0,a, -(ptest:'Supporting'),     ['TestTable','TestObject']),
    tok(1.0,b, -(ptest:'GraspMotion'),    ['TestHand']),
    tok(3.0,c, -(ptest:'Touching'),       ['TestHand','TestObject']),
    tok(4.0,b, +(ptest:'GraspMotion'),    ['TestHand']),
    tok(5.0,a, +(ptest:'Supporting'),     ['TestTable','TestObject']),
    tok(8.0,f, -(ptest:'Supporting'),     ['TestTable','TestObject']),
    tok(9.0,g, -(ptest:'ReleaseMotion'),  ['TestHand']),
    tok(9.5,c, +(ptest:'Touching'),       ['TestHand','TestObject']),
    tok(10.0,g,+(ptest:'ReleaseMotion'),  ['TestHand'])
    ],
    [action(_,ptest:'PickPlace',_,_)]).

test('detect_activity(PickPlace2)', [nondet]) :-
  detect_activity2([
    %%%% first
    tok(0.0,a, -(ptest:'Supporting'),    ['TestTable','TestObject']),
    tok(1.0,b, -(ptest:'GraspMotion'),   ['TestHand']),
    tok(3.0,c, -(ptest:'Touching'),      ['TestHand','TestObject']),
    tok(4.0,b, +(ptest:'GraspMotion'),   ['TestHand']),
    tok(5.0,a, +(ptest:'Supporting'),    ['TestTable','TestObject']),
    tok(8.0,f, -(ptest:'Supporting'),    ['TestTable','TestObject']),
    tok(9.0,g, -(ptest:'ReleaseMotion'), ['TestHand']),
    tok(9.5,c, +(ptest:'Touching'),      ['TestHand','TestObject']),
    tok(10.0,g,+(ptest:'ReleaseMotion'), ['TestHand']),
    %%%% second
    tok(21.0,h, -(ptest:'GraspMotion'),   ['TestHand']),
    tok(23.0,l, -(ptest:'Touching'),      ['TestHand','TestObject']),
    tok(24.0,h, +(ptest:'GraspMotion'),   ['TestHand']),
    tok(25.0,f, +(ptest:'Supporting'),    ['TestTable','TestObject']),
    tok(28.0,z, -(ptest:'Supporting'),    ['TestTable','TestObject']),
    tok(29.0,u, -(ptest:'ReleaseMotion'), ['TestHand']),
    tok(29.5,l, +(ptest:'Touching'),      ['TestHand','TestObject']),
    tok(30.0,u, +(ptest:'ReleaseMotion'), ['TestHand'])
    ],
    [action(_,ptest:'PickPlace',_,_),action(_,ptest:'PickPlace',_,_)]).

test('parser_retract') :-
  test_parser(Parser),
  retractall(parser_grammar(Parser,_,_)),
  rdf_retractall('TestTable',_,_),
  rdf_retractall('TestObject',_,_),
  rdf_retractall('TestHand',_,_).

:- end_tests('knowrob/action_parser').
