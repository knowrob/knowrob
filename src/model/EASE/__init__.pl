
:- tripledb_load('http://www.ease-crc.org/ont/EASE.owl',
    [ graph(tbox),
      namespace(ease)
    ]).

% load modules into user
:- use_module('ACT').
:- use_module('OBJ').
:- use_module('PROC').
:- use_module('STATE').
:- use_module('WF').
:- use_module('EXT').
