/*
  Copyright (C) 2011 Moritz Tenorth, 2020 Daniel Beßler
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

:- module(model_QUDT,
    [
      qudt_type/4,
      qudt_conv/4
    ]).
/** <module> Utilities for handling units of measure and the conversion between different units

@author Moritz Tenorth
@license BSD
*/

%%
%
qudt_type(QUDT_Type,Kind,Multiplier,Offset) :-
  ask( QUDT_Type qudt:quantityKind         Kind ),
  ask( QUDT_Type qudt:conversionMultiplier Multiplier ),
  ask( QUDT_Type qudt:conversionOffset     Offset ),!.

%%
%
qudt_conv(QUDT_TypeI,QUDT_TypeO,ValueI,ValueO) :-
  \+ ground(QUDT_TypeI), !,
  throw(error(instantiation_error, qudt_conv(input_type))).

qudt_conv(QUDT_TypeI,QUDT_TypeO,ValueI,ValueO) :-
  \+ ground(QUDT_TypeO), !,
  throw(error(instantiation_error, qudt_conv(output_type))).

qudt_conv(QUDT_TypeI,QUDT_TypeO,ValueI,ValueO) :-
  \+ ground(ValueI), !,
  throw(error(instantiation_error, qudt_conv(input_value))).
  
qudt_conv(QUDT_Type,QUDT_Type,Value,Value) :- !.
  
qudt_conv(QUDT_TypeI,QUDT_TypeO,ValueI,ValueO) :-
  qudt_type(QUDT_TypeI,Kind,MultiplierI,OffsetI),!,
  qudt_type(QUDT_TypeO,Kind,MultiplierO,OffsetO),
  qudt_conv_(ValueI,MultiplierI,OffsetI,
             ValueO,MultiplierO,OffsetO).
  
qudt_conv(XSD_TypeI,QUDT_TypeO,ValueI,ValueO) :-
  xsd_number_type(XSD_TypeI),!,
  % Input multiplier and offset: assume data to be in the SI base unit
  % corresponding to the OutputType
  MultiplierI is 1.0,
  OffsetI is 0.0,
  qudt_type(QUDT_TypeO,_Kind,MultiplierO,OffsetO),
  qudt_conv_(ValueI,MultiplierI,OffsetI,
             ValueO,MultiplierO,OffsetO).

%%
%
qudt_conv_(ValueI,MultiplierI,OffsetI,
           ValueO,MultiplierO,OffsetO) :-
  number_value_(ValueI,NumI),
  ValueO is (((NumI * MultiplierI + OffsetI) - OffsetO) / MultiplierO).

%%
%
number_value_(Num,Num) :-
  number(Num),!.
number_value_(Atom,Num) :-
  atom_number(Atom,Num),!.
