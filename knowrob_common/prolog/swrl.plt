%%
%% Copyright (C) 2017 by Daniel Beßler
%%
%% This file contains tests for the swrl module
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- begin_tests(swrl).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).

:- owl_parser:owl_parse('package://knowrob_common/owl/swrl_test.owl').

:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

test_rule_id(Id, Descr) :-
  ( rdf_has(Descr, rdfs:label, literal(type(_,Id))) ;
    rdf_has(Descr, rdfs:label, literal(Id)) ),
  rdf_has(Descr, rdf:type, swrl:'Imp').

test(swrl_parse_rules) :-
  forall( rdf_has(Descr, rdf:type, swrl:'Imp'), (
    rdf_swrl_rule(Descr, rule(Head,Body,_)),
    Head \= [], Body \= []
  )).

test_swrl_holds(Id, Bindings) :-
  test_rule_id(Id, Descr),
  rdf_swrl_rule(Descr,rule(Head,Body,Vars)),
  swrl_var_bindings(Vars,Bindings),
  swrl_holds(rule(Head,Body,Vars)).

test_swrl_project(Id, Bindings) :-
  test_rule_id(Id, Descr),
  rdf_swrl_rule(Descr,rule(Head,Body,Vars)),
  swrl_var_bindings(Vars,Bindings),
  swrl_holds(rule(Head,Body,Vars)),
  swrl_project(rule(Head,Body,Vars)).

test(swrl_holds_Person1, [nondet])        :- test_swrl_holds('Person', [['x','http://knowrob.org/kb/swrl_test#Alex']]).
test(swrl_holds_Driver1, [nondet])        :- test_swrl_holds('Driver', [['p','http://knowrob.org/kb/swrl_test#Fred']]).
test(swrl_holds_Driver2, [nondet,fail])   :- test_swrl_holds('Driver', [['p','http://knowrob.org/kb/swrl_test#Ernest']]).
test(swrl_holds_DriverFred, [nondet])     :- test_swrl_holds('DriverFred', []).
test(swrl_holds_greaterThen, [nondet])    :- test_swrl_holds('greaterThen', [['p','http://knowrob.org/kb/swrl_test#Ernest']]),
                                             test_swrl_holds('greaterThen', [['p','http://knowrob.org/kb/swrl_test#Fred']]).
test(swrl_holds_NonHuman1, [nondet])      :- test_swrl_holds('NonHuman', [['x','http://knowrob.org/kb/swrl_test#RedCar']]).
test(swrl_holds_NonHuman2, [nondet,fail]) :- test_swrl_holds('NonHuman', [['x','http://knowrob.org/kb/swrl_test#Fred']]).
test(swrl_holds_brother1, [nondet])       :- test_swrl_holds('brother', [['p','http://knowrob.org/kb/swrl_test#Fred'],
                                                                         ['s','http://knowrob.org/kb/swrl_test#Ernest']]),
                                             test_swrl_holds('brother', [['p','http://knowrob.org/kb/swrl_test#Ernest'],
                                                                         ['s','http://knowrob.org/kb/swrl_test#Fred']]).
test(swrl_holds_brother2, [nondet,fail])  :- test_swrl_holds('brother', [['p','http://knowrob.org/kb/swrl_test#Fred'],
                                                                         ['s','http://knowrob.org/kb/swrl_test#Alex']]).
test(swrl_holds_startsWith, [nondet])     :- test_swrl_holds('startsWith', [['p','http://knowrob.org/kb/swrl_test#Fred']]).
test(swrl_holds_area, [nondet])           :- test_swrl_holds('area', [['r','http://knowrob.org/kb/swrl_test#RectangleBig']]).
test(swrl_holds_bigrect1, [nondet])       :- test_swrl_holds('BigRectangle', [['r','http://knowrob.org/kb/swrl_test#RectangleBig']]).
test(swrl_holds_bigrect2, [nondet,fail])  :- test_swrl_holds('BigRectangle', [['r','http://knowrob.org/kb/swrl_test#RectangleSmall']]).
test(swrl_holds_exactly1, [nondet])       :- test_swrl_holds('exactly', [['x','http://knowrob.org/kb/swrl_test#Lea']]).
test(swrl_holds_exactly2, [nondet,fail])  :- test_swrl_holds('exactly', [['x','http://knowrob.org/kb/swrl_test#Fred']]).

test(swrl_project_Person1, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#Alex',
  \+ owl_individual_of(X, test_swrl:'Person'),
  test_swrl_project('Person',[['x',X]]),
  owl_individual_of(X, test_swrl:'Person').

test(swrl_project_Driver1, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#Fred',
  \+ owl_individual_of(X, test_swrl:'Driver'),
  test_swrl_project('Driver',[['p',X]]),
  owl_individual_of(X, test_swrl:'Driver').

test(swrl_project_NonHuman1, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#RedCar',
  \+ owl_individual_of(X, test_swrl:'NonHuman'),
  test_swrl_project('NonHuman',[['x',X]]),
  owl_individual_of(X, test_swrl:'NonHuman').

test(swrl_project_BigRectangle1, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#RectangleBig',
  \+ owl_individual_of(X, test_swrl:'BigRectangle'),
  test_swrl_project('BigRectangle',[['r',X]]),
  owl_individual_of(X, test_swrl:'BigRectangle'),
  owl_has(X, test_swrl:'hasAreaInSquareMeters', _).

test(swrl_project_startsWith, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#Fred',
  \+ owl_has(X, test_swrl:'hasInternationalNumber', _),
  test_swrl_project('startsWith',[['p',X]]),
  owl_has(X, test_swrl:'hasInternationalNumber', _).

test(swrl_project_exactly, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#Lea',
  \+ owl_individual_of(X, test_swrl:'Singleton'),
  test_swrl_project('exactly',[['x',X]]),
  owl_individual_of(X, test_swrl:'Singleton').

test(swrl_project_hasBrother, [nondet]) :-
  X='http://knowrob.org/kb/swrl_test#Fred',
  Y='http://knowrob.org/kb/swrl_test#Ernest',
  \+ owl_has(X, test_swrl:'hasBrother', _),
  test_swrl_project('brother',[['p',X],['s',Y]]),
  owl_has(X, test_swrl:'hasBrother', Y).
  
:- end_tests(swrl).

