% **********************************************************************
%                                OWL to RDF Generator
% Author: Vangelis Vassiliadis
% Change Log: 
%	  Mar 07: Version 0.5.5: Changes to the use_module and
%	  definitions for Thea 0.5.5 release. 
%         Sep 05: Initial release 0.4 (as part of Thea OWL
%	  Prolog library) 
%             
% **********************************************************************

:- module(owl_as2rdf,
	  [ 	    	    
	    owl_generate_rdf/2, % FileName, RDF_Load_Mode (complete/not)
	    owl_rdf2n3/0 		    
	  ]).

:- use_module('owl_parser.pl').
:- use_module(library('semweb/rdf_db.pl')).

/*
:- use_module(library('semweb/rdf_edit.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('url.pl')).
:- use_module(library('http/http_open.pl')).

*/

/*
owl_generate_rdf(+FileName, +RDF_Load_Mode)
     Top level predicate to generate an RDF/XML FileName from the
     existing OWLAS predicates of the Thea OWL library. If the
     +RDF_Load_Mode is 'complete' then all existing RDF triples are
     first removed.   
*/
owl_generate_rdf(FileName,RDF_Load_Mode) :- 
	(   RDF_Load_Mode=complete,rdf_retractall(_,_,_); true),
	retractall(blanknode_gen(_,_)),
	owl_as2rdf_class,
	owl_as2rdf_subclass,
	owl_as2rdf_disjointSet,
	owl_as2rdf_equivalentSet,
	owl_as2rdf_property,
	owl_as2rdf_individual,
	owl_as2rdf_ontology,
	owl_as2rdf_sameIndividuals,
	owl_as2rdf_differentIndividuals,
	rdf_db:rdf_save(FileName).
	

/*
owl_rdf2n3
     Prints out the RDF triples in N3 notation.   
*/
owl_rdf2n3 :-	
    rdf_db:rdf(S,P,O),
    collapse_ns(S,S1,':',[]),collapse_ns(P,P1,':',[]),collapse_ns(O,O1,':',[]),
    write(S1), write(' '), write(P1), write(' '), write(O1), write(' .'),nl,
    fail.

/*
------------------------------------------------------
owl_as2rdf_class. 
	Generates RDF triples for Class constructs.
*/    

owl_as2rdf_class :-
% C = 'http://www.w3.org/2002/03owlt/miscellaneous/consistent001#WhiteWine',
	class(C,Deprecated,CP,AL,DL),
	owl_rdf_assert(C,'rdf:type','owl:Class'),
	owl_as2rdf_deprecated_class(C,Deprecated),
	owl_as2rdf_triple_list(C,_,AL),
	owl_as2rdf_class_2(C,CP,DL),
	fail.

owl_as2rdf_class.


/*
owl_as2rdf_class_2(+Class, +CP, +DescriptionList).
	Generates RDF triples for Class, depending on the contents 
	of the DescriptionList and the Complete/Partial (CP) indicator
*/    

owl_as2rdf_class_2(C,complete,[unionOf(DL)]) :- !,
	owl_as2rdf_list(DL,Node), 
	owl_rdf_assert(C,'owl:unionOf',Node).

owl_as2rdf_class_2(C,complete,[complementOf(D)]) :- !,
	owl_as2rdf(D,Node), 
	owl_rdf_assert(C,'owl:complementOf',Node).

owl_as2rdf_class_2(C,complete,[intersectionOf(DL)]) :- !,
	owl_as2rdf_list(DL,Node), 
	owl_rdf_assert(C,'owl:intersectionOf',Node).

owl_as2rdf_class_2(C,complete,DL) :- !,
	owl_as2rdf_list(DL,Node),
	owl_rdf_assert(C,'owl:intersectionOf',Node).	

owl_as2rdf_class_2(C,complete,[Description]) :- !,
	owl_as2rdf(Description,Node), 
	owl_rdf_assert(C,'owl:equivalentClass',Node).


owl_as2rdf_class_2(C,partial,DL) :- !,
	owl_as2rdf_triple_list(C,'rdfs:subClassOf',DL).


owl_as2rdf_deprecated_class(C,Deprecated) :-
	( Deprecated = true, !, owl_rdf_assert(C, 'rdf:type','owl:DeprecatedClass'); true).


/*
------------------------------------------------------
owl_as2rdf_subclass. 
	Generates RDF triples for SsubClass constructs.
*/    

owl_as2rdf_subclass :-
	subclassOf(X,Y),
	owl_as2rdf(X,NodeX),owl_as2rdf(Y,NodeY),
	owl_rdf_assert(NodeX,'rdfs:subClassOf',NodeY), 
	fail.

owl_as2rdf_subclass.


/*
------------------------------------------------------
owl_as2rdf_equivalentSet. 
	Generates RDF triples for equivalentSet constructs.
*/    

owl_as2rdf_equivalentSet :-
	equivalentSet(Set),
	owl_as2rdf_equivalentSet_2(Set),
	fail.

owl_as2rdf_equivalentSet.


/*
owl_as2rdf_equivalentSet_2. 
	Generates RDF triples for equivalentClass and equivalentProperty constructs.
*/    
owl_as2rdf_equivalentSet_2([H|T]):-
	(   property(H,_,_,_,_,_,_),!, 
	    owl_as2rdf_set2pairs([H|T],'owl:equivalentProperty')
	    ;  
	    owl_as2rdf_set2pairs([H|T],'owl:equivalentClass')
	).

/*
owl_as2rdf_disjointSet. 
	Generates RDF triples for disjointSet constructs.
*/    

owl_as2rdf_disjointSet :-
	disjointSet(Set),
	owl_as2rdf_set2pairs(Set,'owl:disjointWith'),
	fail.


owl_as2rdf_disjointSet.


/*
------------------------------------------------------
owl_as2rdf_differentIndividuals  
	Generates RDF triples for differentIndividuals constructs.
*/    
owl_as2rdf_differentIndividuals :- 
	differentIndividuals(Set),
	owl_as2rdf_bnode(allDifferent(Set),Node),
	owl_rdf_assert(Node, 'rdf:type', 'owl:AllDifferent'),
        owl_as2rdf_list(Set,Node1),
	owl_rdf_assert(Node,'owl:distinctMembers',Node1),	
	fail.

owl_as2rdf_differentIndividuals.


/*
owl_as2rdf_sameIndividuals  
	Generates RDF triples for sameIndividuals constructs.
*/    
owl_as2rdf_sameIndividuals :- 
	sameIndividuals(Set),
	owl_as2rdf_set2pairs(Set,'owl:sameAs'),	
	fail.

owl_as2rdf_sameIndividuals.


/*
------------------------------------------------------
owl_as2rdf_set2pairs(+Set,+Predicate).
        Given a list (Set) it generates (X Predicate Y) rdf triples
	for all X, Y elements in the list
*/    

owl_as2rdf_set2pairs([],_) :- !.

owl_as2rdf_set2pairs([A|Rest],Predicate) :- !,
	owl_as2rdf_set2pairs(A,Rest,Predicate),
	owl_as2rdf_set2pairs(Rest,Predicate).

/*
owl_as2rdf_set2pairs(+X, +Set,+Predicate).
        It generates (X Predicate Y) rdf triples for all Y elements in
	the list Set. X and Y are converted to RDF Nodes first.
*/    
owl_as2rdf_set2pairs(_,[],_) :- !.
	
owl_as2rdf_set2pairs(A,[B|Rest],Predicate) :- 
	owl_as2rdf(A,Node1),
	owl_as2rdf(B,Node2),
	owl_rdf_assert(Node1,Predicate,Node2),
        owl_as2rdf_set2pairs(A,Rest,Predicate).
	

/*
------------------------------------------------------------------------------------------
owl_as2rdf_property. 
	Generates RDF triples for Property constructs.
*/    
owl_as2rdf_property :- 
	property(PID,Deprecated,AnnotationList,PID_SuperList,
			      PTList,PID_DomainList,PID_RangeList),
	owl_as2rdf_property_2(PID,Deprecated,PTList),
	owl_as2rdf_triple_list(PID,_,AnnotationList),
	owl_as2rdf_triple_list(PID,'rdfs:subPropertyOf',PID_SuperList),
	owl_as2rdf_triple_list(PID,'rdfs:domain',PID_DomainList),
	owl_as2rdf_triple_list(PID,'rdfs:range',PID_RangeList),
	fail.
	
owl_as2rdf_property.

/*
owl_as2rdf_property_2(+PId, +Deprecated, +PropertyTypeSet) 
	Generates RDF triples for property PID based on the type of property
	as defined in the PropertTypeSet options list
*/    
owl_as2rdf_property_2(PID,Deprecated,[OT,F,IF,T,S,iof(Inv)]) :-
	(   Deprecated = true, !, owl_rdf_assert(PID, 'rdf:type','owl:DeprecatedProperty')
	;
	true),
	( nonvar(S),
	  owl_rdf_assert(PID,'rdf:type','owl:SymmetricProperty'),!,Defined = 'yes' ; true),	
	( nonvar(T),owl_rdf_assert(PID,'rdf:type','owl:TransitiveProperty'),
	  !,Defined = 'yes' ; true),
	( nonvar(IF),owl_rdf_assert(PID,'rdf:type','owl:InverseFunctionalProperty'),
	  !,Defined = 'yes' ; true),
	( var(Defined), !,
	  (   OT = objectProperty,owl_rdf_assert(PID,'rdf:type','owl:ObjectProperty'),!;
	      OT = datatypeProperty,owl_rdf_assert(PID,'rdf:type','owl:DatatypeProperty'),!)
	; true),
	(   nonvar(F), owl_rdf_assert(PID,'rdf:type','owl:FunctionalProperty'),!; true),
	(   nonvar(Inv), owl_rdf_assert(PID,'owl:inverseOf',Inv),!; true).

/*
------------------------------------------------------------------------------------------
owl_as2rdf_individual. 
	Generates RDF triples for individual constructs.
*/    
owl_as2rdf_individual :-
	individual(I,AnnotationList,ITList,IVList),
	owl_as2rdf_triple_list(I,_,AnnotationList),
	owl_as2rdf_triple_list(I,'rdf:type',ITList),
	owl_as2rdf_triple_list(I,_,IVList),
	fail.

owl_as2rdf_individual.


/*
------------------------------------------------------------------------------------------
owl_as2rdf_ontology. 
	Generates RDF triples for ontology constructs.
*/           
owl_as2rdf_ontology :-
	ontology(O,OAL),
	owl_rdf_assert(O, 'rdf:type','owl:Ontology'),
	owl_as2rdf_triple_list(O,_,OAL),
	fail.

owl_as2rdf_ontology.


/*
-------------------------------------------------------
owl_as2rdf_triple_list(+ID,+Predicate,+List). 
	Generates RDF triples of the form (ID, Predicate, Y) where Y is 
	each element in List. Nodes are generated for each element Y
	and specific cases for elements Y are handled. Eg Y=type(T) or 
	Y = value(P,V) or Y = annotation(P,V).
*/           

owl_as2rdf_triple_list(_,_,[]).

owl_as2rdf_triple_list(ID,Predicate,[type(T)|Rest]) :- !,
	owl_as2rdf(T,Node),
	owl_rdf_assert(ID,'rdf:type',Node),
	owl_as2rdf_triple_list(ID,Predicate, Rest).

owl_as2rdf_triple_list(ID,Predicate,[value(P,V)|Rest]) :- !,
	owl_as2rdf(P,PNode),
	owl_as2rdf(V,VNode),
	owl_rdf_assert(ID,PNode,VNode),
	owl_as2rdf_triple_list(ID,Predicate, Rest).


% Note: there is no distinction here between OntologyProperty and
% AnnotationProperty. Do we need this?
owl_as2rdf_triple_list(ID,_,[annotation(P,V)|Rest]) :- !,
	owl_rdf_assert(P,'rdf:type','owl:AnnotationProperty'),
	owl_as2rdf(V,VNode),
	owl_rdf_assert(ID,P,VNode), !,
	owl_as2rdf_triple_list(ID,_,Rest).


owl_as2rdf_triple_list(ID,Predicate,[D|Rest]) :-
	owl_as2rdf(D,Node),
	owl_rdf_assert(ID,Predicate,Node),!,
	owl_as2rdf_triple_list(ID,Predicate,Rest).


/*
owl_as2rdf(+Construct, -Node).  
        Generates RDF triples for the Construct based on AS
	transformation rules. If not existing a blankNode is also 
	generated to represent the construct.
        A Construct can be any Description (incl Restrictions), URL, 
	blanknode or literal.
*/   

owl_as2rdf(X,Node) :-
	blanknode_gen(Node,X),!.


owl_as2rdf(unionOf(DL),Node) :- !,
	owl_as2rdf_bnode(unionOf(DL),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Class'),
	owl_as2rdf_list(DL,DLNode),
	owl_rdf_assert(Node,'owl:unionOf',DLNode).
	
owl_as2rdf(intersectionOf(DL),Node) :- !,
	owl_as2rdf_bnode(intersectionOf(DL),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Class'),
	owl_as2rdf_list(DL,Node1),
	owl_rdf_assert(Node,'owl:intersectionOf',Node1).

owl_as2rdf(complementOf(D),Node) :- !,
	owl_as2rdf_bnode(complementOf(D),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Class'),
	owl_as2rdf(D,Node1),
	owl_rdf_assert(Node,'owl:complementOf',Node1).

owl_as2rdf(oneOf(DL),Node) :- !,
	owl_as2rdf_bnode(oneOf(DL),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Class'),
	owl_as2rdf_list(DL,Node1),
	owl_rdf_assert(Node,'owl:oneOf',Node1).

owl_as2rdf(restriction(PropertyID,allValuesFrom(Descr)),Node) :-  !,
	owl_as2rdf_bnode(restriction(PropertyID,allValuesFrom(Descr)),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Restriction'),
	owl_rdf_assert(Node,'owl:onProperty',PropertyID),
	owl_as2rdf(Descr,Node1),
	owl_rdf_assert(Node,'owl:allValuesFrom',Node1).

owl_as2rdf(restriction(PropertyID,someValuesFrom(Descr)),Node) :- !,
	owl_as2rdf_bnode(restriction(PropertyID,someValuesFrom(Descr)),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Restriction'),
	owl_rdf_assert(Node,'owl:onProperty',PropertyID),
	owl_as2rdf(Descr,Node1),
	owl_rdf_assert(Node,'owl:someValuesFrom',Node1).

owl_as2rdf(restriction(PropertyID,value(Value)),Node) :- !,
	owl_as2rdf_bnode(restriction(PropertyID,value(Value)),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Restriction'),
	owl_rdf_assert(Node,'owl:onProperty',PropertyID),
	owl_as2rdf(Value,Node1),
	owl_rdf_assert(Node,'owl:hasValue',Node1).

owl_as2rdf(restriction(PropertyID,minCardinality(Min)),Node) :- !,
	owl_as2rdf_bnode(restriction(PropertyID,minCardinality(Min)),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Restriction'),
	owl_rdf_assert(Node,'owl:onProperty',PropertyID),
	owl_rdf_assert(Node,'owl:minCardinality',Min).

owl_as2rdf(restriction(PropertyID,maxCardinality(Max)),Node) :- !,
	owl_as2rdf_bnode(restriction(PropertyID,maxCardinality(Max)),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Restriction'),
	owl_rdf_assert(Node,'owl:onProperty',PropertyID),
	owl_rdf_assert(Node,'owl:maxCardinality',Max).

owl_as2rdf(restriction(PropertyID,cardinality(Card)),Node) :- !,
	owl_as2rdf_bnode(restriction(PropertyID,cardinality(Card)),Node),
	owl_rdf_assert(Node,'rdf:type','owl:Restriction'),
	owl_rdf_assert(Node,'owl:onProperty',PropertyID),
	owl_rdf_assert(Node,'owl:cardinality',Card).

owl_as2rdf('rdfs:Literal','rdfs:Literal') :-!.

owl_as2rdf(literal(X),literal(X)):-!.

owl_as2rdf(ID, ID) :- !.


/*
owl_as2rdf_list(+List, -Node).  
        Generates RDF triples for the List of construct based on
	Abstract Syntax list transformation rules. Node represents the 
	List in the resulting RDF graph
*/   
owl_as2rdf_list([],'rdf:nil').

owl_as2rdf_list([S|Rest],Node) :-
	owl_as2rdf_bnode([S|Rest],Node),
	owl_rdf_assert(Node,'rdf:type','rdf:List'),
	owl_as2rdf(S,Ts),
	owl_rdf_assert(Node,'rdf:first', Ts),	
	owl_as2rdf_list(Rest,Node2),
	owl_rdf_assert(Node,'rdf:rest',Node2).

/*
owl_rdf_assert(S,P,O).  
        Expands the NS the S, P, O terms and asserts into the RDF
	database
*/   
owl_rdf_assert(S,P,O) :- 
	expand_ns(S,S1),expand_ns(P,P1),expand_ns(O,O1),
	rdf_db:rdf_assert(S1,P1,O1), !.

/*
owl_as2rdf_bnode(+X,-Node).  
        It generates a bnode Node for construct X in case it does not
	exist already as a blanknode/3 clause.
*/   
owl_as2rdf_bnode(X,Node) :-
	blanknode(Node,X,_),
	assert(blanknode_gen(Node,X)),!.

owl_as2rdf_bnode(X,Node) :-
	rdf_db:rdf_bnode(Node),
	assert(blanknode_gen(Node,X)),!.



