:- begin_rdf_tests(
    'srdl',
    'package://knowrob/owl/robots/PR2.owl',
    [ namespace('http://knowrob.org/kb/PR2.owl#')
    ]).

:- use_module('./srdl.pl').

:- end_tests('srdl').
