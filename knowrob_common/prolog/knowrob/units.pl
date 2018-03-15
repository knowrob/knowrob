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

:- module(knowrob_units,
    [
      convert_to_unit/3
    ]).
/** <module> Utilities for handling units of measure and the conversion between different units

@author Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).

:- rdf_meta(convert_to_unit(r,r,r)).

:- owl_parse('package://knowrob_common/owl/knowrob_units.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_units, 'http://knowrob.org/kb/knowrob_units.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(qudt, 'http://qudt.org/schema/qudt#', [keep(true)]).
:- rdf_db:rdf_register_ns(unit, 'http://qudt.org/vocab/unit#', [keep(true)]).
:- rdf_db:rdf_register_ns(quantity, 'http://qudt.org/vocab/quantity#', [keep(true)]).
:- rdf_db:rdf_register_ns(qudt-dimension, 'http://qudt.org/vocab/dimension#', [keep(true)]).
:- rdf_db:rdf_register_ns(qudt-dimensionalunit, 'http://qudt.org/1.1/vocab/dimensionalunit#', [keep(true)]).

% This hook provides transparent integration of the conversion routine
% into the query process. If the user queries for results in a given unit, and
% there are results in a different, but compatible unit (e.g. meters and
% centimeters), they are converted and returned.
rdfs_computable:rdfs_computable_triple_during(Property, Frame, literal(type(OutputType, OutputVal)), Interval) :-
  % if value is partially given (e.g. literal(type(unit:Meter, Val)) ),
  % try to call the respective computable without the binding and to
  % convert the results into the correct form
  ground([Property,Frame,OutputType]),
  rdf_triple_during(Property, Frame, TempVal, Interval),
  % try to convert the results into the specified format
  convert_to_unit_internal(TempVal, OutputType, OutputVal).

convert_to_unit_internal(literal(type(Type, _)), Type, _) :- fail, !.
convert_to_unit_internal(literal(type(InputType, InputVal)), OutputType, Res) :-
  convert_to_unit(literal(type(InputType, InputVal)), OutputType, Res).

%% convert_to_unit(+Input, +OutputType, ?Result) is semidet.
%
% True if Result is the value of Input converted to OutputType.
% 
convert_to_unit(literal(type(InputType, InputVal)), OutputType, Res) :-
  % check if values can be converted at all (equal quantity kind)
  rdf_has(InputType,  qudt:quantityKind, QKind), % TODO: OR: input float or double
  rdf_has(OutputType, qudt:quantityKind, QKind),
  % get input multiplier and offset
  rdf_has_prolog(InputType, qudt:conversionMultiplier, MultI),
  rdf_has_prolog(InputType, qudt:conversionOffset,     OffI),
  % get output multiplier and offset
  rdf_has_prolog(OutputType, qudt:conversionMultiplier, MultO),
  rdf_has_prolog(OutputType, qudt:conversionOffset,     OffO),
  term_to_atom(I,  InputVal),
  Res is (((I * MultI + OffI) - OffO) / MultO).

% default case: input is given as standard xsd type, assume SI base unit
convert_to_unit(Input, OutputType, Res) :-
  (Input = literal(type('http://www.w3.org/2001/XMLSchema#double', InputVal)) ;
   Input = literal(type('http://www.w3.org/2001/XMLSchema#float',  InputVal)) ),
  % Input multiplier and offset: assume data to be in the SI base unit
  % corresponding to the OutputType
  MultI=1.0,
  OffI=0.0,
  % get output multiplier and offset
  rdf_has_prolog(OutputType, qudt:conversionMultiplier, MultO),
  rdf_has_prolog(OutputType, qudt:conversionOffset,     OffO),
  term_to_atom(I,  InputVal),
  Res is (((I * MultI + OffI) - OffO) / MultO).
