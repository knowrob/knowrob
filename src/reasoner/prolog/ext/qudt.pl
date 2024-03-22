:- module(model_qudt,
    [ qudt_unit/4,
      qudt_conv/4
    ]).
/** <module> Utilities for handling units of measure and the conversion between different units

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'), [ rdf/4, rdf_load/2 ]).

% register RDF namespace of QUDT ontology
:- sw_register_prefix(qudt, 'http://data.nasa.gov/qudt/owl/qudt#').

% load QUDT RDF data into a graph named "qudt"
:- rdf_load('../../../../owl/unit.owl', [graph(qudt), silent(true)]).


%% qudt_unit(?Symbol,?Kind,?Multiplier,?Offset) is nondet.
%
% Facts about units in the QUDT model.
%
% @param Symbol SI symbol of the unit
% @param Kind the quantity kind
% @param Multiplier multiplication factor for conversion
% @param Offset offset for conversion
%
qudt_unit(Symbol, Kind, Multiplier, Offset) :-
	rdf(Type, qudt:quantityKind,         Kind,                         qudt),
	rdf(Type, qudt:symbol,               literal(type(_,Symbol)),      qudt),
	rdf(Type, qudt:conversionMultiplier, literal(type(_,MultiplierA)), qudt),
	rdf(Type, qudt:conversionOffset,     literal(type(_,OffsetA)),     qudt),
	atom_number(MultiplierA,Multiplier),
	atom_number(OffsetA,Offset).

%% qudt_conv(+SymbolI,+SymbolO,+ValueI,-ValueO) is semidet.
%
% Convert a value to another unit of the same kind.
%
% @param SymbolI SI unit symbol of input
% @param SymbolO SI unit symbol of output
% @param ValueI the input value
% @param ValueO the output value
%
qudt_conv(SymbolI,_SymbolO,_ValueI,_ValueO) :-
	\+ ground(SymbolI), !,
	throw(error(instantiation_error, qudt_conv(input_type))).

qudt_conv(_SymbolI,SymbolO,_ValueI,_ValueO) :-
	\+ ground(SymbolO), !,
	throw(error(instantiation_error, qudt_conv(output_type))).

qudt_conv(_SymbolI,_SymbolO,ValueI,_ValueO) :-
	\+ ground(ValueI), !,
	throw(error(instantiation_error, qudt_conv(input_value))).
  
qudt_conv(Symbol,Symbol,Value,Value) :- !.
  
qudt_conv(SymbolI,SymbolO,ValueI,ValueO) :-
	qudt_unit(SymbolI,Kind,MultiplierI,OffsetI), !,
	qudt_unit(SymbolO,Kind,MultiplierO,OffsetO),
	qudt_conv1(
		ValueI,MultiplierI,OffsetI,
		ValueO,MultiplierO,OffsetO).
  
qudt_conv(XSD_TypeI,SymbolO,ValueI,ValueO) :-
	xsd_number_type(XSD_TypeI),
	!,
	% Input multiplier and offset: assume data to be in the SI base unit
	% corresponding to the OutputType
	MultiplierI is 1.0,
	OffsetI is 0.0,
	qudt_unit(SymbolO,_Kind,MultiplierO,OffsetO),
	qudt_conv1(
		ValueI,MultiplierI,OffsetI,
		ValueO,MultiplierO,OffsetO).

%%
qudt_conv1(ValueI,MultiplierI,OffsetI,
           ValueO,MultiplierO,OffsetO) :-
	(	number(ValueI) -> NumI
	;	atom_number(ValueI,NumI)
	),
	ValueO is (((NumI * MultiplierI + OffsetI) - OffsetO) / MultiplierO).
