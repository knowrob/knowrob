%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module provides utilities for handling OWL information
%% in KnowRob.
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


:- module(knowrob_units,
    [
      convert_to_unit/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).

:- rdf_meta(convert_to_unit(r,r,r)).

% :- owl_parser:owl_parse('../owl/knowrob_units.owl', false, false, true).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).

:- rdf_db:rdf_register_ns(knowrob_units, 'http://ias.cs.tum.edu/kb/srdl2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(qudt, 'http://qudt.org/schema/qudt#', [keep(true)]).
:- rdf_db:rdf_register_ns(unit, 'http://qudt.org/vocab/unit#', [keep(true)]).
:- rdf_db:rdf_register_ns(qudt-dimension, 'http://qudt.org/vocab/dimension#', [keep(true)]).
:- rdf_db:rdf_register_ns(qudt-dimensionalunit, 'http://qudt.org/1.1/vocab/dimensionalunit#', [keep(true)]).





convert_to_unit(Input, OutputType, Res) :-

    % TODO: check if values can be converted at all (both lengths?)

    % get input value
    Input = literal(type(InputType, InputVal)),

    % get input multiplier
    rdf_has(InputType, qudt:conversionMultiplier, literal(type(_,InputConvMult))),

    % get output multiplier
    rdf_has(OutputType, qudt:conversionMultiplier, literal(type(_,OutputConvMult))),

    term_to_atom(I,  InputVal),
    term_to_atom(CI, InputConvMult),
    term_to_atom(CO, OutputConvMult),

    Res is (I * CI / CO). % TODO: not only scaling, also shifting



:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-

    % if value is partially given (e.g. literal(type(unit:Meter, Val)) ), 
    % try to call the respective computable without the binding and to
    % convert the results into the correct form

    nonvar(Value),
    Value = literal(type(OutputType, OutputVal)),
    rdf_triple(Property, Frame, TempVal),

    % try to convert the results into the specified format
    convert_to_unit(TempVal, OutputType, OutputVal))

  , X), assert(user:(X)).







