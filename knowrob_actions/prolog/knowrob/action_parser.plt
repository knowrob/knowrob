:- begin_tests('knowrob/action_parser').

:- use_module(library('knowrob/action_parser')).

:- dynamic test_parser/1.

test('parser_assert') :-
  parser_create(Parser),
  assertz(test_parser(Parser)),
  rdf_assert('TestTable', rdf:type, dul:'PhysicalArtifact'),
  rdf_assert('TestObject', rdf:type, dul:'PhysicalArtifact'),
  rdf_assert('TestHand', rdf:type, ease:'PhysicalEffector').

% FIXME: Opening/Closing is skipped!!!
%          -> do not skip if some state constituent exists

test('detect_activity(PickingUp)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0, a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0, b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0, c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0, d, +(motions:'GraspingMotion'),           ['TestHand'])
    ],
    [_,action(actions:'PickingUp',_,_)]).

test('detect_activity(not PickingUp)', [fail]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(0.5,a, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,d, +(motions:'GraspingMotion'),           ['TestHand'])
    ],
    [_,action(actions:'PickingUp',_,_)]).

test('detect_activity(GraspLift)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,a,-(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b,-(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c,-(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,d,+(motions:'GraspingMotion'),           ['TestHand']),
    tok(5.0,f,+(force_dynamics:'SupportingContact'), ['TestTable','TestObject'])
    ],
    [_,action(actions:'GraspLift',_,_)]).

test('detect_activity(Placing)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(0.9,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(3.0,c, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,b, +(motions:'ReleasingMotion'),          ['TestHand'])
    ],
    [_,action(actions:'Placing',_,_)]).

test('detect_activity(PickPlace)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    tok(0.0,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,b, +(motions:'GraspingMotion'),           ['TestHand']),
    tok(5.0,a, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(8.0,f, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(9.0,g, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(9.5,c, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(10.0,g,+(motions:'ReleasingMotion'),          ['TestHand'])
    ],
    [_,action(actions:'PickPlace',_,_)]).

test('detect_activity(PickPlace2)', [nondet]) :-
  test_parser(Parser),
  detect_activity2(Parser,[
    %%%% first
    tok(0.0,a, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(1.0,b, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(3.0,c, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(4.0,b, +(motions:'GraspingMotion'),           ['TestHand']),
    tok(5.0,a, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(8.0,f, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(9.0,g, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(9.5,c, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(10.0,g,+(motions:'ReleasingMotion'),          ['TestHand']),
    %%%% second
    tok(21.0,h, -(motions:'GraspingMotion'),           ['TestHand']),
    tok(23.0,l, -(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(24.0,h, +(motions:'GraspingMotion'),           ['TestHand']),
    tok(25.0,f, +(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(28.0,z, -(force_dynamics:'SupportingContact'), ['TestTable','TestObject']),
    tok(29.0,u, -(motions:'ReleasingMotion'),          ['TestHand']),
    tok(29.5,l, +(force_dynamics:'EffectorContact'),   ['TestHand','TestObject']),
    tok(30.0,u, +(motions:'ReleasingMotion'),          ['TestHand'])
    ],
    [_,action(actions:'PickPlace',_,_),action(actions:'PickPlace',_,_)]).

test('parser_retract') :-
  test_parser(Parser),
  retractall(parser_grammar(Parser,_,_)),
  rdf_retractall('TestTable',_,_),
  rdf_retractall('TestObject',_,_),
  rdf_retractall('TestHand',_,_).

:- end_tests('knowrob/action_parser').
