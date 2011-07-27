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


:- module(knowrob_owl,
    [
      convert_to_unit/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).

:- rdf_meta(convert_to_unit(r,r,r)).


convert_to_unit(Input, OutputType, Res) :-

    % get input value
    Input = literal(type(InputType, InputVal)),

    % get input multiplier
    rdf_has(InputType, 'http://qudt.org/schema/qudt#conversionMultiplier', literal(type(_,InputConvMult))),

    % get output multiplier
    rdf_has(OutputType, 'http://qudt.org/schema/qudt#conversionMultiplier', literal(type(_,OutputConvMult))),


    term_to_atom(I,  InputVal),
    term_to_atom(CI, InputConvMult),
    term_to_atom(CO, OutputConvMult),

    Res is (I * CI / CO).




