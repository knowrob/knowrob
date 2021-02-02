:- module(model_QUDT,
    [ qudt_unit/4,
      qudt_conv/4
    ]).
/** <module> Utilities for handling units of measure and the conversion between different units

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf/4, rdf_load/2, rdf_register_ns/3 ]).

% register RDF namespace of QUDT ontology
:- rdf_register_ns(qudt,
	'http://data.nasa.gov/qudt/owl/qudt#', [keep(true)]).

% load QUDT RDF data into a graph named "qudt"
:- rdf_load('../../owl/unit.owl', [graph(qudt)]).


%% qudt_unit(?Symbol,?Kind,?Multiplier,?Offset) is nondet.
%
% Facts about units in the QUDT model.
%
% @Symbol SI symbol of the unit
% @Kind the quantity kind
% @Multiplier multiplication factor for conversion
% @Offset offset for conversion
%
qudt_unit(Symbol, Kind, Multiplier, Offset) :-
	rdf(Type, qudt:quantityKind,         Kind,                         qudt),
	rdf(Type, qudt:symbol,               literal(type(_,Symbol)),      qudt),
	rdf(Type, qudt:conversionMultiplier, literal(type(_,MultiplierA)), qudt),
	rdf(Type, qudt:conversionOffset,     literal(type(_,OffsetA)),     qudt),
	atom_number(MultiplierA,Multiplier),
	atom_number(OffsetA,Offset).

%%
% Define SI unit symbols as operator.
% FIXME: this is a bit risky, I think. Could there be an operator clash?
%
:- forall(
		qudt_unit(Symbol,_,_,_),
		op(1000, xf, user:(Symbol))).

%% qudt_conv(+SymbolI,+SymbolO,+ValueI,-ValueO) is semidet.
%
% Convert a value to another unit of the same kind.
%
% @SymbolI SI unit symbol of input
% @SymbolO SI unit symbol of output
% @ValueI the input value
% @ValueO the output value
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
