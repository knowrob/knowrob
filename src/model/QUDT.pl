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
    [ rdf/4,
      rdf_load/2,
      rdf_unload_graph/1,
      rdf_register_ns/3
    ]).
:- use_module(library('http/http_open.pl'),
    [ http_open/3 ]).

:- rdf_register_ns(qudt,
    'http://data.nasa.gov/qudt/owl/qudt#', [keep(true)]).
% TODO load into DB backend
%:- load_owl('http://...',
    %[ graph(qudt),
      %namespace(qudt)
    %]).

:- dynamic qudt_unit/4.

%%
rdf_qudt_unit(Symbol,Kind,Multiplier,Offset) :-
  rdf(Type, qudt:quantityKind,         Kind,                         qudt ),
  rdf(Type, qudt:symbol,               literal(type(_,Symbol)),      qudt ),
  rdf(Type, qudt:conversionMultiplier, literal(type(_,MultiplierA)), qudt ),
  rdf(Type, qudt:conversionOffset,     literal(type(_,OffsetA)),     qudt ),
  atom_number(MultiplierA,Multiplier),
  atom_number(OffsetA,Offset).

%%
%
qudt_conv(SymbolI,_SymbolO,_ValueI,_ValueO) :-
	\+ ground(SymbolI),
	!,
	throw(error(instantiation_error, qudt_conv(input_type))).

qudt_conv(_SymbolI,SymbolO,_ValueI,_ValueO) :-
	\+ ground(SymbolO),
	!,
	throw(error(instantiation_error, qudt_conv(output_type))).

qudt_conv(_SymbolI,_SymbolO,ValueI,_ValueO) :-
	\+ ground(ValueI),
	!,
	throw(error(instantiation_error, qudt_conv(input_value))).
  
qudt_conv(Symbol,Symbol,Value,Value) :- !.
  
qudt_conv(SymbolI,SymbolO,ValueI,ValueO) :-
	qudt_unit(SymbolI,Kind,MultiplierI,OffsetI),
	!,
	qudt_unit(SymbolO,Kind,MultiplierO,OffsetO),
	qudt_conv_(
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
	qudt_conv_(
		ValueI,MultiplierI,OffsetI,
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

%%
qudt_init_ :-
	% TODO load into some RDF graph seperated from the rest
	rdf_load('../../owl/unit.owl',[graph(qudt)]),
	%%
	forall(
		rdf_qudt_unit(Symbol,Kind,Multiplier,Offset),
		(	op(1000, xf, user:(Symbol)),
			assertz( qudt_unit(Symbol,Kind,Multiplier,Offset) )
		)
	),
	%rdf_unload_graph(qudt)
	true.

:- qudt_init_.
