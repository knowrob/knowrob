/*
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
*/

:- module(knowrob_math_expressions,
    [
      eval_owl_term/2
    ]).
/** <module> Utilities for handling units of measure and the conversion between different units

@author Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).

:-  rdf_meta
    eval_owl_term(r,?).

:- owl_parse('package://knowrob_common/owl/math-expressions.owl').

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
