/** <module> Utilities for handling units of measure and the conversion between different units

  Copyright (C) 2011 Moritz Tenorth
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Moritz Tenorth
@license BSD

*/

:- module(knowrob_math,
    [
      eval_owl_term/2,
      quaternion_multiply/3,
      parse_vector/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).

:- rdf_meta(eval_owl_term(r,?)).

:- owl_parse('package://knowrob_common/owl/knowrob_math.owl').

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).




eval_owl_term(OWL, Val) :-
    rdf_has(OWL, rdf:type, knowrob:'ValueTerm'),
    rdf_has(OWL, knowrob:termvalue, literal(type(xsd:double,ValAtom))),
    term_to_atom(Val, ValAtom).


eval_owl_term(OWL, Val) :-
    rdf_has(OWL, rdf:type, knowrob:'AdditionTerm'),
    rdf_has(OWL, knowrob:op1, Op1),
    rdf_has(OWL, knowrob:op2, Op2),
    
    eval_owl_term(Op1, Val1),
    eval_owl_term(Op2, Val2),
    
    Val is Val1 + Val2.
    

eval_owl_term(OWL, Val) :-
    rdf_has(OWL, rdf:type, knowrob:'SubtractionTerm'),
    rdf_has(OWL, knowrob:op1, Op1),
    rdf_has(OWL, knowrob:op2, Op2),
    
    eval_owl_term(Op1, Val1),
    eval_owl_term(Op2, Val2),
    
    Val is Val1 - Val2.
    
    
eval_owl_term(OWL, Val) :-
    rdf_has(OWL, rdf:type, knowrob:'MultiplicationTerm'),
    rdf_has(OWL, knowrob:op1, Op1),
    rdf_has(OWL, knowrob:op2, Op2),
    
    eval_owl_term(Op1, Val1),
    eval_owl_term(Op2, Val2),
    
    Val is Val1 * Val2.

    
eval_owl_term(OWL, Val) :-
    rdf_has(OWL, rdf:type, knowrob:'DivisionTerm'),
    rdf_has(OWL, knowrob:op1, Op1),
    rdf_has(OWL, knowrob:op2, Op2),
    
    eval_owl_term(Op1, Val1),
    eval_owl_term(Op2, Val2),
    
    Val is Val1 / Val2.

parse_vector([X|Y], [X|Y]).
parse_vector(In, Numbers) :-
  parse_vector(In, Numbers, ' ').
parse_vector(In, Numbers, Delimiter) :-
  atom(In),
  atomic_list_concat(Atoms, Delimiter, In),
  findall(Num, (
    member(Atom,Atoms),
    atom_number(Atom,Num)
  ), Numbers),
  length(Atoms,L),
  length(Numbers,L).
  %jpl_call('org.knowrob.utils.MathUtil', 'parseVector', [In, ' '], OutArr),
  %not(OutArr = @(null)),
  %jpl_array_to_list(OutArr, Out).

quaternion_multiply(Q0,Q1,Multiplied) :-
  jpl_list_to_array(Q0, Q0_array),
  jpl_list_to_array(Q1, Q1_array),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionMultiply', [Q0_array, Q1_array], Out_array),
  jpl_array_to_list(Out_array, Multiplied).
