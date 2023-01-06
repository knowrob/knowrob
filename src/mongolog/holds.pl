:- module(mongolog_holds,
	[ holds(t),    % ?Query
	  holds(r,t,t) % ?Subject, ?Predicate, ?Object
	]).
/** <module> The *holds* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('mongodb/client'),
	[ mng_strip_operator/3, mng_strip_type/3 ]).
:- use_module(library('qudt'),
	[ qudt_unit/4 ]).
:- use_module('mongolog_test').
:- use_module(library('scope')).
:- use_module('temporal').

:- dynamic holds/3.
:- multifile holds/3.

%%
% Enforce arithmetic operator.
%
arithmetic_operator(=,is) :- !.
arithmetic_operator(X,X).

%%
% Strip unit from value term for compile-time conditions.
% Only succeeds if unit information is provided in Term.
%
strip_unit(Term, Operator, Multiplier, Offset, Value) :-
	nonvar(Term),
	% strip opearator if any
	mng_strip_operator(Term, MngOperator, WithoutOperator),
	compound(WithoutOperator),
	% try to gather unit data
	WithoutOperator =.. [Symbol, WithoutUnit],
	qudt_unit(Symbol, _Kind, Multiplier, Offset),
	% make sure Operator is a known command that can be called
	arithmetic_operator(MngOperator, Operator),
	% remove type info (must be numeric)
	mng_strip_type(WithoutUnit, _, Value).

%% holds(?Subject, ?Property, ?Value) is nondet.
%
% Query values of a property on some subject.
% In case of datatype properties, the value may be a term `Operator(Value)`
% where Operator is a comparison operator (e.g. "<"), meaning that only triples
% are yielded where the comparison between actual value and Value yields true.
%
% @param Subject The subject of a triple.
% @param Property The predicate of a triple.
% @param Value The object of a triple.
%
holds(Subject, Property, Value) ?>
	% this clause is only used if the O argument is instantiated
	% to a term that contains unit information.
	% Else this clause will be pruned by the compiler (caused by pragma/1 commands).
	% The idea is that we call triple/3 with a fresh variable for O,
	% and then perform any arithmetic operations only after the value has
	% been converted to the requested unit.
	pragma(mongolog_holds:strip_unit(Value, Operator,
		Multiplier, Offset, Stripped)),
	% replace O with a new variable BaseUnitValue
	triple(Subject, Property, BaseUnitValue),
	% convert BaseUnitValue to requested unit
	Converted is (BaseUnitValue - Offset) / Multiplier,
	% perform arithmetic operation defined by Operator
	call(Operator, Stripped, Converted).

holds(Subject, Property, Value) ?>
	% make sure that either this or above clause are compiled
	% into a hold/3 query, never both
	pragma(\+ mongolog_holds:strip_unit(Value,_,_,_,_)),
	triple(Subject, Property, Value).

holds(Subject, Property, Value) +>
	pragma(mongolog_holds:strip_unit(Value, 'is',
		Multiplier, Offset, Stripped)),
	% convert to base unit
	BaseUnitValue is ((Stripped * Multiplier) + Offset),
	% store in base unit
	triple(Subject, Property, BaseUnitValue).

holds(Subject, Property, Value) +>
	pragma(\+ mongolog_holds:strip_unit(Value,_,_,_,_)),
	triple(Subject, Property, Value).

%% holds(+Query) is nondet.
%
% Same as holds/3 with arguments wrapped into a single
% term `Property(Subject,Value)`.
%
% @param Query the query term.
%
holds(Query) ?+>
	pragma(Query =.. [P,S,O]),
	holds(S,P,O).

     /*******************************
     *	    UNIT TESTS	     		    *
     *******************************/

:- begin_mongolog_tests(
	'mongolog_holds',
	'owl/test/swrl.owl',
	[ namespace('http://knowrob.org/kb/swrl_test#')
	]).

test('holds/1 with ns', [ blocked('holds/1 cannot handle namespaces') ]) :-
	assert_true(holds(test:'hasHeightInMeters'(test:'RectangleBig',13))).

test('holds(+S,+P,+O)') :-
	assert_true(holds(test:'Ernest', test:'hasSibling', test:'Fred')).

test('project(holds(+S,+P,+O))') :-
    universal_scope(Scope),
	assert_false(holds(test:'Lea', test:'hasNumber', '+493564754647')),
	assert_true(mongolog_call(project(holds(test:'Lea', test:'hasNumber', '+493564754647')), [scope(Scope)])),
	assert_true(holds(test:'Lea', test:'hasNumber', '+493564754647')).

test('holds(+S,+P,+Unit(+O))') :-
    universal_scope(Scope),
	assert_false(holds(test:'Lea',test:'hasHeightInMeters', _)),
	assert_true(mongolog_call(project(holds(test:'Lea',test:'hasHeightInMeters', m(6.5))), [scope(Scope)])),
	assert_true(holds(test:'Lea',test:'hasHeightInMeters', cm(650))),
	assert_true(holds(test:'Lea',test:'hasHeightInMeters', cm(650.0))),
	assert_false(holds(test:'Lea',test:'hasHeightInMeters', cm(750.0))),
	assert_false(holds(test:'Lea',test:'hasHeightInMeters', cm(600.0))),
	assert_true(holds(test:'Lea',test:'hasHeightInMeters', cm(_))),
	holds(test:'Lea',test:'hasHeightInMeters', cm(X)) -> assert_equals(X,650.0); fail.

:- end_mongolog_tests('mongolog_holds').
