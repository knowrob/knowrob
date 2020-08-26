
:- tripledb_load('http://www.ease-crc.org/ont/SOMA.owl',
    [ namespace(soma)
    ]).

% load modules into user
:- use_module('ACT').
:- use_module('OBJ').
:- use_module('PROC').
:- use_module('STATE').
:- use_module('WF').
:- use_module('IO').
:- use_module('EXT').
