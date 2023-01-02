:- module(mongolog_rdfs,
    [ is_resource(r),
      is_property(r),
      is_literal(r),
      is_datatype(r),
      has_type(r,r),
      has_range(r,r),
      has_domain(r,r),
      has_label(r,?),
      has_comment(r,?),
      rdf_list(r,t),
      instance_of(r,r),   % ?Individual, ?Class
      subclass_of(r,r),   % ?Class, ?SuperClass
      subproperty_of(r,r) % ?Property, ?SuperProperty
    ]).
/** <module> The Resource Description Framework Schema model.

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf_register_ns/3,
	  rdf_meta/1,
	  rdf_current_ns/2,
	  rdf_split_url/3
	]).
:- use_module(library('mongolog/mongolog')).
:- use_module(library('rdftest')).
:- use_module(library('scope')).

:- multifile instance_of/2, subclass_of/2, subproperty_of/2.
:- dynamic instance_of/2, subclass_of/2, subproperty_of/2.

:- rdf_register_ns(rdfs,
	'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).

:- rdf_meta(expand_list(r,t,t)).

%:- multifile instance_of/2.
%:- multifile subclass_of/2.
%:- multifile subproperty_of/2.

%% has_type(+Resource,?Type) is semidet.
%
% rdf:type is an instance of rdf:Property that is used to
% state that a resource is an instance of a class.
%
% @param Resource a RDF resource
% @param Type a rdf:type of the resource
%
has_type(Resource,Type) ?>
	% TODO: this seems not a good idea, e.g. Type could be a regex/1 term.
	pragma(\+ compound(Type)),
	triple(Resource, rdf:type, Type).

has_type(Resource,Type) +>
	triple(Resource, rdf:type, Type).

%% is_resource(+Entity) is semidet.
%
% All things described by RDF are called resources,
% and are instances of the class rdfs:Resource.
% This is the class of everything.
% All other classes are subclasses of this class.
% rdfs:Resource is an instance of rdfs:Class.
%
% @param Entity An entity IRI.
%
is_resource(Entity) ?+>
	has_type(Entity, rdfs:'Resource').

%% is_property(+Entity) is semidet.
%
% rdf:Property is the class of RDF properties.
% rdf:Property is an instance of rdfs:Class.
%
% @param Entity An entity IRI.
%
is_property(Entity) ?+>
	has_type(Entity, rdf:'Property').

%% is_literal(+Entity) is semidet.
%
% The class rdfs:Literal is the class of literal values such as
% strings and integers. Property values such as textual strings
% are examples of RDF literals.
%
% rdfs:Literal is an instance of rdfs:Class.
% rdfs:Literal is a subclass of rdfs:Resource.
%
% @param Entity An entity IRI.
%
is_literal(Entity) ?+>
	has_type(Entity, rdfs:'Literal').

%% is_datatype(+Entity) is semidet.
%
% rdfs:Datatype is the class of datatypes.
% All instances of rdfs:Datatype correspond to the RDF model
% of a datatype described in the RDF Concepts specification.
% rdfs:Datatype is both an instance of and a subclass of rdfs:Class.
% Each instance of rdfs:Datatype is a subclass of rdfs:Literal.
%
% @param Entity An entity IRI.
%
is_datatype(Entity) ?+>
	has_type(Entity, rdfs:'Datatype').

%% has_range(?Property,?Range) is nondet.
%
% The range of a property globally restricts values
% of the property to instances of the range.
%
% @param Property a property
% @param Range the range of the property
%
has_range(Property,Range) ?+>
	triple(Property, rdfs:range, Range).

%% has_domain(?Property,?Domain) is nondet.
%
% The domain of a property globally restricts hosts
% of the property to instances of the domain.
%
% @param Property a property
% @param Domain the range of the property
%
has_domain(Property,Domain) ?+>
	triple(Property, rdfs:domain, Domain).

%% has_label(+Resource,?Comment) is semidet.
%
% rdfs:label is an instance of rdf:Property that may be used
% to provide a human-readable version of a resource's name.
%
% @param Resource a RDF resource
% @param Label a label atom
%
has_label(Resource,Label) ?+>
	annotation(Resource, rdfs:label, Label).

%% has_comment(+Resource,?Comment) is semidet.
%
% rdfs:comment is an instance of rdf:Property that may be used
% to provide a human-readable description of a resource.
%
% @param Resource a RDF resource
% @param Comment a comment atom
%
has_comment(Resource,Comment) ?>
	annotation(Resource, rdfs:comment, Comment).

%% instance_of(?Entity,?Type) is nondet.
%
% The type of an entity (rdf:type).
% For example: `Nibbler instance_of Cat`.
%
% Note: that the *projection* clause of this rule allows
% Entity to be a variable, in which case a new entity
% symbol is generated.
%
% @param Entity a named individual
% @param Type the type of the entity
%
instance_of(A,B) ?+>
	triple(A,rdf:type,B).

%% subclass_of(?Class,?SuperClass) is nondet.
%
% The subclass-of relation (rdfs:subClassOf).
% For example: `Cat subclass_of Animal`.
%
% @param Class a class IRI
% @param SuperClass a class IRI
%
subclass_of(A,B) ?+>
	triple(A, rdfs:subClassOf, B).

%% subproperty_of(?Property,?SuperProperty) is nondet.
%
% The subproperty-of relation (rdfs:subPropertyOf).
%
% @param Property a property IRI
% @param SuperProperty a property IRI
%
subproperty_of(A,B) ?+>
	triple(A, rdfs:subPropertyOf, B).

%%
expand_list(rdf:nil, [], []) :- !.

expand_list(This, [Child|Rest],
		[	triple(This, rdf:first, Child),
			triple(This, rdf:rest, Next)
		|	Xs
		]) :-
	expand_list(Next, Rest, Xs).

%% rdf_list(+RDF_list, -Pl_List) is semidet.
%
% Read a RDF list into a Prolog list.
%
rdf_list(RDF_list, Pl_List) ?>
	var(Pl_List),
	ground(RDF_list),
	findall(X,
		(	triple(RDF_list, reflexive(transitive(rdf:rest)), Ys),
			triple(Ys, rdf:first, X)
		),
		Pl_List).

rdf_list(RDF_list, Pl_list) ?>
	pragma(Pl_list = [First|_]),
	ground(First),
	triple(RDF_list, rdf:first, First),
	findall(X,
		(	triple(RDF_list, reflexive(transitive(rdf:rest)), Ys),
			triple(Ys, rdf:first, X)
		),
		Pl_list).

rdf_list(RDF_list, Pl_List) +>
	pragma((
		ground(Pl_List),
		model_RDFS:expand_list(RDF_list, Pl_List, Expanded)
	)),
	call([
		has_type(RDF_list, rdf:'List')
	|	Expanded
	]).

%%
rdf_list_head(SubList, ListHead) ?>
	findall(X,
		(	X = SubList
		;	triple(X, transitive(rdf:rest), SubList)
		),
		ListHeads),
	length(ListHeads,NumHeads),
	Index is NumHeads-1,
	nth0(Index, ListHeads, ListHead).

		 /*******************************
		 *	    PORTRAYING TERMS	  	*
		 *******************************/

%%
% user:portray clauses are used to display terms
% in a human readable fashion to the user.
%
user:portray(Term) :-
	convert_(Term,Term0),
	write(Term0).

convert_(Term,Term0) :-
	% recursively call convert_ on compound terms
	compound(Term),!,
	Term=..[Fun|Args],
	findall(Arg0,
		(	member(Arg1,Args),
			convert_(Arg1,Arg0)
		),
		Args0
	),
	Term0=..[Fun|Args0].

convert_(A,B) :-
	% read description for class names starting with underscore.
	% this is the case e.g. for OWL restriction classes
	(atom(A);string(A)),
	atom_concat('_',_,A),
	mongolog_call(has_description(A,B0)),
	B0 \= class(_),!,
	convert_(B0,B).

convert_(A,:(Prefix,NameEnquoted)) :-
	% convert full IRI to short notation if possible
	atom(A),
	rdf_split_url(URI,Name,A),
	rdf_current_ns(Prefix,URI),
	atom_chars(Name,[HeadC|_]),
	(	char_type(HeadC,lower)
	->	NameEnquoted=Name
	;	atomic_list_concat(['\'',Name,'\''],'',NameEnquoted)
	),
	!.

convert_(A,A).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- begin_rdf_tests(
		'mongolog_rdfs',
		'owl/test/swrl.owl',
		[ namespace('http://knowrob.org/kb/swrl_test#')
		]).

test('is_resource(+Resource)') :-
	assert_true(is_resource(test:'Adult')),
	assert_false(is_resource(test:'Lea')),
	assert_false(is_resource(test:'hasNumber')),
	assert_false(is_resource(test:'NotExisting')).

test('is_property(+Property)') :-
	assert_true(is_property(test:'hasNumber')),
	assert_false(is_property(test:'Lea')),
	assert_false(is_property(test:'NotExisting')).

test("instance_of(+,+)") :-
	universal_scope(QScope),
	assert_true(instance_of(test:'Rex', test:'Man')),
	assert_false(instance_of(test:'Rex', test:'Adult')),
	assert_true(mongolog_call(project(instance_of(test:'Rex', test:'Adult')), [scope(QScope)])),
	assert_true(instance_of(test:'Rex', test:'Adult')).

test("subproperty_of(+Sub,+Sup)") :-
	assert_true(subproperty_of(test:'hasParent', test:'hasAncestor')),
	assert_false(subproperty_of(test:'hasBrother', test:'hasSibling')),
	universal_scope(QScope),
	assert_true(mongolog_call(project(subproperty_of(test:'hasBrother', test:'hasSibling')), [scope(QScope)])),
	assert_true(subproperty_of(test:'hasBrother', test:'hasSibling')).

test_list(RDF_list) :-
	mongolog_call(triple(test:testchain, owl:propertyChainAxiom, RDF_list)).

test('rdf_list(+,+)') :-
	test_list(RDF_list),
	assert_true(rdf_list(RDF_list, [test:hasParent,test:hasAncestor])),
	assert_false(rdf_list(RDF_list, [test:hasAncestor,test:hasParent])),
	assert_false(rdf_list(RDF_list, [test:hasParent])).

test('rdf_list(+,-)') :-
	test_list(RDF_list),
	assert_true(rdf_list(RDF_list, _)),
	(	rdf_list(RDF_list, Pl_List)
	->	assert_equals(Pl_List, [test:hasParent,test:hasAncestor])
	;	true
	).

test('rdf_list(-,+)') :-
	test_list(RDF_list1),
	assert_true(rdf_list(_, [test:hasParent,test:hasAncestor])),
	(	rdf_list(RDF_list2, [test:hasParent,test:hasAncestor])
	->	assert_equals(RDF_list1, RDF_list2)
	;	true
	).

test('rdf_list(+,-),length(+,-)') :-
	mongolog_call((
		triple(test:testchain, owl:propertyChainAxiom, RDF_list),
		rdf_list(RDF_list, List),
		length(List, NumElems)
	)),
	assert_equals(NumElems, 2).

test('rdf_list_head(+,-)') :-
	test_list(RDF_list1),
	mongolog_call(triple(SubList, rdf:first, test:hasAncestor)),
	assert_true(mongolog_call(rdf_list_head(SubList, _))),
	(	mongolog_call(rdf_list_head(SubList, RDF_list2))
	->	assert_equals(RDF_list2, RDF_list1)
	;	true
	).

:- end_rdf_tests('mongolog_rdfs').
