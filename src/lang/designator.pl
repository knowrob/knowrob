:- module(lang_designator,
	[ is_designator(t),
	  has_designator(r,t)
	]).
/** <module> Implementation of entity designators.

These are descriptions of entities that are independent
of their identity.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf_equal/2 ]).

%% is_designator(+Designator) is semidet.
%
% True for instantiated designators.
%
is_designator(X)  :- var(X), !, fail.
is_designator([a,_|_])  :- !.
is_designator([an,_|_]) :- !.

%% has_designator(?Entity,+Designator) is nondet.
%
% Find entities denoted by some designator.
%
has_designator(Entity, Designator) :-
	ground(Designator), !,
	% get conjunction of statements
	get_designator_statements(Entity,Designator,Statements),
	% issue a query
	lang_query:ask(Statements).

%%
get_designator_statements(Entity,Designator,Statements) :-
	(	member([iri,IRI],Designator)
	->	Entity=IRI
	;	true
	),
	%%
	bagof(X,
		get_designator_statement0(Entity,Designator,X),
		Statements
	).

%%
get_designator_statement0(Entity,[A,Type|Designator],Statement) :-
	once(A=a;A=an),
	(	get_designator_statement1(Entity,Designator,Statement)
	;	get_type_statement(Entity,Type,Statement)
	).

%%
get_designator_statement1(Entity,Designator,Statement) :-
	member([Key,Value],Designator),
	\+ Key=iri,
	get_designator_property(Key,Property),
	get_designator_statement2(Entity,Property,Value,Statement).

%%
get_designator_statement2(Entity,Property,NestedDesignator,Statement) :-
	is_designator(NestedDesignator),
	%%
	(	member([iri,IRI],NestedDesignator)
	->	NestedEntity=IRI
	;	NestedEntity=_
	),
	%%
	!,
	(	get_designator_statement0(NestedEntity,NestedDesignator,Statement)
	;	Statement=holds(Entity,Property,NestedEntity)
	).

get_designator_statement2(Entity,Property,AtomicValue,
		holds(Entity,Property,Value)) :-
	get_designator_value(Property,AtomicValue,Value).

%%
get_type_statement(Entity,DesignatorType,triple(Entity,Property,RDFType)) :-
	rdf_equal(rdf:type, Property),
	get_iri(DesignatorType,RDFType,camelcase).

%%
get_designator_property(DesignatorKey,RDFProperty) :-
	get_iri(DesignatorKey,RDFProperty,lower_camelcase).

%%
get_designator_value(_Property,DesignatorValue,RDFValue) :-
%	is_object_property(Property),
%	!,
	get_iri(DesignatorValue,RDFValue,camelcase),
	!.

get_designator_value(_Property,Value,Value) :-
	!.

%%
get_iri(Var, Var, _) :-
	var(Var),
	!.

get_iri(IRI, IRI, _Formatter) :-
	atom(IRI),
	atom_concat('http://',_,IRI),
	!.

get_iri(Name, regex(Pattern), Formatter) :-
	atom(Name),
	call(Formatter, Name, RDFName),
	% query for the full IRI using regex
	atomic_list_concat(['^.*#',RDFName,'$'], '', Pattern).

     /*******************************
     *	    UNIT TESTS	     		    *
     *******************************/

:- begin_rdf_tests(
    'lang_designator',
    'package://knowrob/owl/test/swrl.owl',
    [ namespace('http://knowrob.org/kb/swrl_test#')
    ]).

:- use_module(library('semweb/rdf_db'),
    [ rdf_meta/1 ]).

:- rdf_meta(test_designator(r,t,t)).

test_designator(Entity,Designator,Expected) :-
	get_designator_statements(Entity,Designator,Statements),
	assert_equals(Statements,Expected).

test('designator type IRI') :-
	test_designator(X,
		[ an, test:'Adult' ],
		[ triple(X,rdf:type,test:'Adult') ]
	).

test('designator type name') :-
	test_designator(X,
		[ an, adult ],
		[ triple(X,rdf:type,regex('^.*#Adult$')) ]
	).

test('designator property') :-
	test_designator(X,
		[ an, adult, [ has_age, 70] ],
		[ holds(X,regex('^.*#hasAge$'), 70),
		  triple(X,rdf:type,regex('^.*#Adult$'))
		]
	).

test('designator nested') :-
	test_designator(X,
		[ an, adult, [ has_parent,
			[ an, adult, [ iri, Part ]]
		]],
		[ triple(Part,rdf:type,regex('^.*#Adult$')),
		  holds(X,regex('^.*#hasParent$'),Part),
		  triple(X,rdf:type,regex('^.*#Adult$'))
		]
	).

test('has-designator') :-
	assert_true(has_designator(test:'Lea',[a,man])),
	assert_true(has_designator(test:'Lea',[a,woman])),
	assert_true(has_designator(test:'Lea',[a,woman,[has_parent,fred]])),
	assert_false(has_designator(test:'Lea',[a,car])).

:- end_rdf_tests('lang_designator').
