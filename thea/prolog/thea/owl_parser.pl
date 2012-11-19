% **********************************************************************
%                                OWL PARSER
% Author: Vangelis Vassiliadis
% Change Log:
%             May 04: Initial release 0.1
%             Jan 05: Version 0.2 - Some code optimisation.
%             Feb 05: DisjointClass, EquivalentClass
%             Mar 05: DifferentFrom
%
% Version 0.3 First release, March 22, 2005
%
% Version 0.4 Changes (use of SWI's rdf parser 5.5.12)
%             * rdf_load/2 options to noshare of blank nodes and use a
%               convert function to hold datatyped values.
%             * removed fix_owl. Apparently is not needed with SWI 5.5.x
%	      * Implemented equivalentProperties by re-using the logic
%	        of equivalentClass and sameAs.
%	      * Implementation of owl:imports (handled at RDF/triple
%		level) by importing all RDF triples prior to building
%		the the OWL abstract syntax terms. Flag to select if
%		imports will be handled.
%	      * OWL parser can parse now either local files or URLs (in-
%		line with Semweb's package RDF parser). Use of SWI's
%		http package.
%	      * owl_pack_ontology, owl_report
% Version 0.5.5: March 07: Changes to the use_module and definitions for Thea 0.5.5 release.
% To do        Check and report for purely Internal (DL errors)
%              Inferences
%
% **********************************************************************

:- module(owl_parser,
	  [ convert/3,
	    owl_parse/4,
	    owl_parse/1,
	    owl_report/0,
	    owl_pack_ontology/0,
	    expand_ns/2,                  %  ?NS_URL, ?Full_URL
	    collapse_ns/4,


	    class/5,			  % Export all the dynamic ones also
	    owl/4,
	    subclassOf/2,
	    property/7,
	    annotationProperty/1,
	    differentIndividuals/1,
	    sameIndividuals/1,
	    ontology/2,
	    individual/4,
	    disjointSet/1,
	    equivalentSet/1,
	    blanknode/3,
	    blanknode_gen/2,

	    owl_parser_log/2
	  ]).


:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdf_edit.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('url.pl')).
:- use_module(library('http/http_open.pl')).

:- dynamic(class/5).
:- dynamic(owl/4).
:- dynamic(subclassOf/2).
:- dynamic(property/7).
:- dynamic(annotationProperty/1).
:- dynamic(differentIndividuals/1).
:- dynamic(sameIndividuals/1).
:- dynamic(ontology/2).
:- dynamic(individual/4).
:- dynamic(disjointSet/1).
:- dynamic(equivalentSet/1).
:- dynamic(blanknode/3).
:- dynamic(owl_parser_log/2).

:- dynamic(blanknode_gen/2).
:- dynamic(ontology/12).

:- dynamic(outstream/1).

% -----------------------------------------------------------------------
%                                Top Level  Predicates
% -----------------------------------------------------------------------

% owl_parse: Top level predicate to parse a set of RDF triples
%  	     and produce an Abstract Syntax representation of an
%	     OWL ontology.
%
% owl_parse(+URL, +RDF_Load_Mode, +OWL_Parse_Mode, +Imports)
%	Calls the rdf_load_stream predicate to parse RDF stream in URL.
%       If RDF_Load_Mode = complete it first retacts all rdf triples.
%       If Imports = true it handles owl:import clause at RDF level.
%
% owl_parse(+OWL_Parse_Mode).


owl_parse(URL, RDF_Load_Mode, OWL_Parse_Mode,Imports) :-
	(   RDF_Load_Mode=complete,!,rdf_retractall(_,_,_); true),
	retractall(rdf_db:rdf_source(_,_,_,_)),
	rdf_load_stream(URL,[URL],Imports),
	owl_parse(OWL_Parse_Mode).


owl_parse(OWL_Parse_Mode) :-
	rdf_2_owl,
	(   OWL_Parse_Mode=complete,!,owl_clear_as; true),
	owl_count(Z0), owl_parse_named_classes, owl_count(Z1), Z01 is Z0 - Z1,
	owl_parser_log(['Getting named classes...', Z01, ' triples used']),
	owl_parse_subclasses, owl_count(Z2), Z12 is Z1 - Z2,
	owl_parser_log(['Getting subclasses...',Z12, ' triples used']),
	owl_parse_disjoint_classes, owl_count(Z3), Z23 is Z2 - Z3,
	owl_parser_log(['Getting disjoint sets of classes...', Z23, ' triples used']),
	owl_parse_equivalent_classes, owl_count(Z4), Z34 is Z3 - Z4,
	owl_parser_log(['Getting equivalent sets of classes...', Z34, ' triples used']),
	owl_parse_property, owl_count(Z5), Z45 is Z4 - Z5,
	owl_parser_log(['Getting properties...', Z45, ' triples unused']),
	owl_parse_annotationProperty, owl_count(Z6),Z56 is Z5 - Z6,
	owl_parser_log(['Getting Annotation properties...',Z56,' triples used']),
        owl_parse_individual_axioms, owl_count(Z7), Z67 is Z6 - Z7,
	owl_parser_log(['Getting Individual axioms...',Z67, 'triples used']),
	owl_parse_ontology, owl_count(Z8), Z78 is Z7 - Z8,
	owl_parser_log(['Getting Ontology...',Z78, ' triples used ']),
	owl_parse_named_individuals,owl_count(Z9), Z89 is Z8 - Z9,
	owl_parser_log(['Getting named Individuals...',Z89, ' triples used']),
	owl_parse_unnamed_classes, owl_count(Z10), Z910 is Z9 - Z10,
	owl_parser_log(['Getting unamed classes...', Z910, ' triples used']),
	owl_parse_unnamed_individuals,owl_count(Z11), Z1011 is Z10 - Z11,
	owl_parser_log(['Getting unnamed Individuals...',Z1011, 'triples used']),
	owl_parser_log(['Nubber of unused remain triples ', Z11]).


owl_pack_ontology :-
	ontology(O,OA),
	findall(class(C1,C2,C3,C4,C5), class(C1,C2,C3,C4,C5),CL),
	findall(subclassOf(S1,S2), subclassOf(S1,S2),SL),
	findall(property(P1,P2,P3,P4,P5,P6,P7),property(P1,P2,P3,P4,P5,P6,P7),PL),
	findall(annotationProperty(AP1),annotationProperty(AP1),APL),
	findall(differentIndividuals(DI),differentIndividuals(DI),DIL),
	findall(sameIndividuals(SI),sameIndividuals(SI),SIL),
	findall(individual(I1,I2,I3,I4), individual(I1,I2,I3,I4), IL),
	findall(disjointSet(DS), disjointSet(DS), DSL),
	findall(equivalentSet(ES),equivalentSet(ES),ESL),
	findall(blanknode(BN1,BN2,BN3),blanknode(BN1,BN2,BN3),BNL),
	assert(ontology(O,OA,CL,SL,PL,APL,DIL,SIL,IL,DSL,ESL,BNL)).

owl_report :-
	ontology(_O,OA,CL,SL,PL,APL,DIL,SIL,IL,DSL,ESL,BNL),
	length(OA, OACount), print('Ontology Properties:'-OACount), nl,
	length(CL, CLCount), print('Classes:'-CLCount), nl,
	length(SL, SLCount), print('SubClasses:'-SLCount), nl,
	length(PL, PLCount), print('Properties:'-PLCount), nl,
	length(APL, APLCount), print('Annotation Properties:'-APLCount), nl,
	length(DIL, DILCount), print('Different Individuals'-DILCount), nl,
	length(SIL, SILCount), print('Same Individuals'-SILCount), nl,
	length(IL, ILCount), print('Individuals'-ILCount), nl,
        length(DSL, DSLCount), print('Disjoint Classes'-DSLCount), nl,
        length(ESL, ESLCount), print('Equivalent Classes'-ESLCount), nl,
        length(BNL, BNLCount), print('Blank Nodes'-BNLCount), nl.


% -----------------------------------------------------------------------
%                                UTILITY Predicates
% -----------------------------------------------------------------------

%%       owl_parser_log(+Log)
%
%       Log is a list; together with a timestamp it is asserted as
%       an owl_parser_log/2 term.

owl_parser_log(Log) :-
	get_time(T),convert_time(T,TS),
	assertz(owl_parser_log(TS, Log)).


%%       owl_clear_as.
%
%       Clears the prolog terms that store the Abstract Syntax
%       implementation of the OWL ontology.

owl_clear_as :-
	retractall(class(_,_,_,_,_)),
	retractall(subclassOf(_,_)),
	retractall(property(_,_,_,_,_,_,_)),
	retractall(annotationProperty(_)),
	retractall(differentIndividuals(_)),
	retractall(sameIndividuals(_)),
	retractall(ontology(_,_)),
	retractall(individual(_,_,_,_)),
	retractall(disjointSet(_)),
	retractall(equivalentSet(_)),
	retractall(blanknode(_,_,_)),
	retractall(ontology(_,_,_,_,_,_,_,_,_,_,_,_)).




convert(T,V,typed_value(T,V)).


%%	rdf_2_owl.
%
%       Converts RDF triples to OWL/4 triples so that
%	their use can tracked by the OWL parser.
rdf_2_owl :-
	owl_parser_log(['Removing existing owl triples']),
	retractall(owl(_,_,_,_)),
	owl_parser_log('Copying RDF triples to OWL triples'),
	rdf(X,Y,Z),
%	owl_fix_no(X,X1), owl_fix_no(Y,Y1), owl_fix_no(Z,Z1),
	assert(owl(X,Y,Z,not_used)), fail.

rdf_2_owl :-
	owl_count(Z),
	owl_parser_log(['Number of owl triples copied: ',Z]).


%% rdf_load_stream(+URL, +ImportedList)
%
%	This predicate calls the rdf parser to parse the RDF/XML URL
%	into RDF triples. URL can be a local file or a URL.
%	The predicate recursively calls itself for all URLs that need to
%	be imported, ie. are objects to an owl:imports predicate.
%	The ImportedList argument contains the imported so far URLs,
%	to avoid re-visiting the same URLs. (Empty List in 1st call).
%
rdf_load_stream(URL,Imported,Imports) :-
  (sub_string(URL,0,4,_,'http'), !,
    http_open(URL,RDF_Stream,[]),
    rdf_load(RDF_Stream,[blank_nodes(noshare)]),
    close(RDF_Stream)
    ;
    RDF_Stream = URL, % URL is a file name
    rdf_load(RDF_Stream,[blank_nodes(noshare)])
  ),
	(   Imports = true,
	    rdf(_,'http://www.w3.org/2002/07/owl#imports',Import_URL),
	    not( member(Import_URL, Imported)),!,
            rdf_load_stream(Import_URL,[Import_URL|Imported],Imports)
	  ; true).


%	fix_no(+A,-B)
%
%	This is used to correct an RDF parser error:
%       To remove duplicate ## from a URL.

/*	Obsolete with version 5.5.x of SWI's RDF parser
owl_fix_no(literal(X), literal(X)) :- !.

owl_fix_no(A,B) :-
	sub_atom(A,Start,_,After,'##'),
	sub_atom(A,0,Start,_,New_A),
	Start1 is Start + 2,
	sub_atom(A,Start1,After,_,B1),
	atom_concat(New_A,'#',A1),
	atom_concat(A1,B1,B),!,print(A-B).

owl_fix_no(A,A).
*/


%	owl_count(?U).
%       Returns/Checks the number of unused OWL triples.

owl_count(U) :-
	findall(1,owl(_,_,_,not_used),X), length(X,U).


%       use_owl(?S,?P,?O)
%	Marks an OWL triple as used. Expands the S,P,O.

use_owl(X1,Y1,Z1) :-
	expand_ns(X1,X),
	expand_ns(Y1,Y),
	expand_ns(Z1,Z),
	owl(X,Y,Z, not_used),
	retract(owl(X,Y,Z, not_used)),
	assert(owl(X,Y,Z,used)).


%	use_owl(?S,?P,?O,named).
%
%       Same as use_owl/3, but marks only if S 	is Named URI (i.e. non blank node).

use_owl(X1,Y1,Z1,named) :-
	expand_ns(X1,X),
	expand_ns(Y1,Y),
	expand_ns(Z1,Z),
	owl(X,Y,Z, not_used),
	not(sub_string(X,0,2,_,'__')),
	retract(owl(X,Y,Z, not_used)),
	assert(owl(X,Y,Z,used)).


%       expand_ns(+NS_URL, ?Full_URL)
%
%       Expands a 'namespaced' URI of the form ns:fragment to a full URI
%       substituting the full expansion for ns from the ns/2 facts

expand_ns(NS_URL, Full_URL) :-
	nonvar(NS_URL),
	not(NS_URL = literal(_)),
	uri_split(NS_URL,Short_NS,Term, ':'),
	rdf_db:ns(Short_NS,Long_NS),!,
	concat_atom([Long_NS,Term],Full_URL).

expand_ns(URL, URL).


%       collapse_ns(+FullURL, ?NSURL, +Options)
%
%	Collapses a full URI of the form Path#fragment to a Namespaced
%	URI NS:fragment substituting the full expansion for ns from
%	the ns/2 facts
%	Char is either ':' for normal ns notation or '_' for builing
%	prolog terms.
%	Options supported: no_base(ShortNs): Use only term!

collapse_ns(FullURL, NSURL,Char,Options) :-
	nonvar(FullURL),
	not(FullURL = literal(_)),
	uri_split(FullURL,LongNS, Term, '#'),
	concat(LongNS,'#',LongNS1),
	rdf_db:ns(ShortNS,LongNS1),
	(   member(no_base(ShortNS),Options), ! , NSURL = Term
	;
	concat_atom([ShortNS,Char,Term],NSURL)
	),!.

collapse_ns(URL, URL,_,_).



%       uri_split(+URI,-Namespace,-Term,+Split_Char) :-
%
%       Splits a URI into the Namespace and the Term parts
%       separated by the Split_Char character.
%       It supposes URI = concat(Namespace,Split_Char,Term)

uri_split(URI,Namespace,Term,Split_Char) :-
	sub_atom(URI,Start,_,After,Split_Char),
	sub_atom(URI,0,Start,_,Namespace),
	Start1 is Start + 1,
	sub_atom(URI,Start1,After,_,Term).


%       owl_collect_linked_nodes(+Node,+Predicate, +InList,-OutList)

%	Appends Node to the InList, and recursively, all other
%	Nodes that are linked with the Predicate to the Node. The
%	result is returned to OutList.

owl_collect_linked_nodes(Node,Predicate,InList,OutList) :-
	use_owl(Node,Predicate,A),!,
	owl_collect_linked_nodes(Node,Predicate,InList,List1),
	owl_collect_linked_nodes(A,Predicate,List1,OutList).

owl_collect_linked_nodes(Node,Predicate,InList,OutList) :-
	use_owl(A,Predicate,Node),!,
	owl_collect_linked_nodes(Node,Predicate,InList,List1),
	owl_collect_linked_nodes(A,Predicate,List1,OutList).

owl_collect_linked_nodes(Node,_,List, [Node|List]) :-
	not(memberchk(Node, List)),!.

owl_collect_linked_nodes(_,_,List, List) :- !.


% ----------------------------------------------------------------
%                OWL Parser implementation predicates
% ----------------------------------------------------------------

%       owl_deprecated_class(+CID,-Deprecated).
%
%       Deprecated is set to true if Class CID is defined as deprecated.
%       false otherwise.

owl_deprecated_class(CID,Deprecated) :-
	use_owl(CID,'rdf:type','owl:DeprecatedClass'), Deprecated = true,!;
	Deprecated = false.

%       owl_deprecated_property(+PID,-Deprecated).
%
%	Deprecated is set to true if Property PID is defined as
%	deprecated; false otherwise.

owl_deprecated_property(PID,Deprecated) :-
	use_owl(PID,'rdf:type','owl:DeprecatedProperty'), Deprecated = true,!;
	Deprecated = false.

%       owl_get_bnode(+Node,+Description)
%
%	if Node is a blank (not named) node, then it is asserted in
%	the database as a blanknode(Node,Description,used) term.
%	The purpose is to record when a blank node has been used, so
%	subsequent uses of it will result in structure sharing.

owl_get_bnode(Node,Description) :-
	sub_string(Node,0,2,_,'__'),!,
	not( blanknode(Node,_,_)),
	assert(blanknode(Node,Description, used)).

owl_get_bnode(_,_).

%       owl_optional_type(+D).
%
%	It simply consumes any optional owl:Class or
%	rdfs:Class type triples for description D

use_optional_type(D) :-
  use_owl(D,'rdf:type','owl:Class'), use_owl(D,'rdf:type','rdfs:Class'),!;
  use_owl(D,'rdf:type','rdfs:Class'),!;
  true.


%       owl_description_list(+Node, -List)
%
%       If +Node is defined as rdf:type rdf:List, then List returns
%       a prolog list of descriptions for this Node.

owl_description_list('http://www.w3.org/1999/02/22-rdf-syntax-ns#nil',[]) :- !.

owl_description_list(X,[F|R]) :-
	use_owl(X,'rdf:type','rdf:List'),
	use_owl(X,'rdf:first',Element),
	owl_description(Element,F),
	use_owl(X,'rdf:rest',Y),
	!,owl_description_list(Y,R).


%       owl_individual_list(+Node, -List)
%
%       If +Node is defined as rdf:type rdf:List, then List returns
%       a prolog list of individuals for this Node.

owl_individual_list('http://www.w3.org/1999/02/22-rdf-syntax-ns#nil',[]) :- !.

owl_individual_list(X,[F|R]) :-
	use_owl(X,'rdf:type','rdf:List'),
	use_owl(X,'rdf:first',F),
	use_owl(X,'rdf:rest',Y),
	!,owl_individual_list(Y,R).


%       owl_restriction(+Element,-Restriction).
%
%       If Element is defined as a owl:Restriction on property P then
%       Restriction binds to a restriction(Property,Type) term,
%	according to OWL Abstract syntax specification.

owl_restriction(Element,Restriction) :-
	use_owl(Element,'rdf:type','owl:Restriction'),
	use_owl(Element, 'owl:onProperty',PropertyID),
	owl_restriction_type(Element,PropertyID, Restriction),!.

owl_restriction_type(E, PropertyID, restriction(PropertyID,allValuesFrom(Descr))) :-
	use_owl(E, 'owl:allValuesFrom',D),
	owl_description(D, Descr),!.

owl_restriction_type(E, PropertyID, restriction(PropertyID,someValuesFrom(Descr))) :-
	use_owl(E, 'owl:someValuesFrom',D),
	owl_description(D, Descr),!.

owl_restriction_type(E, PropertyID,restriction(PropertyID,cardinality(C))) :-
	use_owl(E, 'owl:cardinality',C).

owl_restriction_type(E, PropertyID,restriction(PropertyID,minCardinality(C))) :-
	use_owl(E, 'owl:minCardinality',C).

owl_restriction_type(E, PropertyID, restriction(PropertyID,maxCardinality(C))) :-
	use_owl(E, 'owl:maxCardinality',C).
owl_restriction_type(E, PropertyID, restriction(PropertyID,value(Value))) :-
	use_owl(E, 'owl:hasValue',Value).


% ----------------------------------------------------------------------
%       owl_description(+Node,-Description).
%
%	It implements OWL AS production rules for Descriptions.
%       I.e. a Description can be any of
%         - a Class ID
%	  - an existing blank node (in which case we have structure
%	    sharing),
%         - a unionOf(DescriptionList) term.
%         - a intersectionOf(DescriptionList) term.
%         - a complementOf(Description) term.
%         - a oneOf(IndividualList) term.
%
%         During the construction of the Description any blank node
%         is recorded for later structure sharing checks.

owl_description(C,C) :-
	not(sub_string(C,0,2,_,'__')).

owl_description(C,D) :-
	blanknode(C,D,Use),
	(   Use = used, owl_parser_log(C-D),
	    retractall(blanknode(C,D,used)),
	    assert(blanknode(C,D,shared))
	;
	    true).

owl_description(D,Restriction) :-
	owl_restriction(D, Restriction),
	use_optional_type(D),
	owl_get_bnode(D,Restriction).

owl_description(D,unionOf(L)) :-
	use_owl(D,'owl:unionOf',L1),
	use_optional_type(D),
	owl_description_list(L1,L),
	owl_get_bnode(D,unionOf(L)).

owl_description(D,intersectionOf(L)) :-
	use_owl(D,'owl:intersectionOf',L1),
	use_optional_type(D),
	owl_description_list(L1,L),
	owl_get_bnode(D,intersectionOf(L)).

owl_description(D,complementOf(Descr)) :-
	use_owl(D,'owl:complementOf',D1),
	use_optional_type(D),
	owl_description(D1,Descr),
	owl_get_bnode(D,complementOf(Descr)).

owl_description(D,oneOf(L)) :-
	use_owl(D,'owl:oneOf',L1),
	use_optional_type(D),
	owl_individual_list(L1,L),
	owl_get_bnode(D,oneOf(L)).

% --------------------------------------------------------------------
%                        Classes and Subclasses
%
%       owl_parse_named_classes
%
%       Any named node defined as an owl:Class or rdfs:Class is
%       asserted int the database as a class/5 term with all
%       Descriptions and annotations defined for this Class ID
%       Note that the construction of a class term cannot be done
%       incrementally, i.e. we cannot add descriptions or annotations
%       to an existing class.

owl_parse_named_classes :-
	(   use_owl(C,'rdf:type','owl:Class',named) ; use_owl(C,'rdf:type','rdfs:Class',named)),
	owl_deprecated_class(C,Deprecated),
	not(class(C,_,_,_,_)),
        findall(D,(owl_description(C,D),C \= D), DL),
	findall(CA,owl_annotation(C,CA), CAL),
	assert(class(C,Deprecated,complete,CAL, DL)),
	(   use_owl(C, 'rdf:type','rdfs:Class'); true ),
	fail.

owl_parse_named_classes.


%       owl_parse_unnamed_classes
%
%       Same as above for unnamed Classes. (Not in OWL DL)

owl_parse_unnamed_classes :-
	(   use_owl(C,'rdf:type','owl:Class') ; use_owl(C,'rdf:type','rdfs:Class')),
	owl_deprecated_class(C,Deprecated),
	not(class(C,_,_,_,_)),
        findall(D,(owl_description(C,D),C \= D ), DL),
	findall(CA,(owl_annotation(C,CA)), CAL),
	assert(class(C,Deprecated,complete,CAL, DL)),
	(   use_owl(C, 'rdf:type','rdfs:Class'); true ),
	fail.

owl_parse_unnamed_classes.


%       owl_parse_subclasses
%
%	Asserts a subclassOf(DescriptionX,DescriptionY) term for each
%       X rdfs:subClassOf Y triple.

owl_parse_subclasses :-
	use_owl(X,'rdfs:subClassOf',Y),
        owl_description(X,DX),
	owl_description(Y,DY),
        assert(subclassOf(DX,DY)),fail.

owl_parse_subclasses.


% --------------------------------------------------------------------
%                           Classes Axioms
%               (EquivalentClasses, DisjointClasses)


%       owl_parse_equivalent_classes.
%
%	Asserts an equivalentSet(DescriptionList) term for each set
%	of connected with owl:equivalentClass Nodes.
%	DescriptionList is a list of Descriptions for these Nodes.

owl_parse_equivalent_classes:-
	use_owl(X,'owl:equivalentClass',Y),
	owl_collect_linked_nodes(X,'owl:equivalentClass',[],List1),
	owl_collect_linked_nodes(Y,'owl:equivalentClass',List1,List2),
	owl_set_descriptions(List2,SetD),
	assert(equivalentSet(SetD)),
	fail.

owl_parse_equivalent_classes.


%       owl_parse_disjoint_classes.
%
%	Constructs the Disjoint Sets(Set) for nodes that are
%	connected with owl:disjointWith links.


owl_parse_disjoint_classes :-
	findall(X-Y,use_owl(X,'owl:disjointWith',Y),Arcs),
	owl_disjoint_arcs(Arcs).


%	owl_disjoint_arcs(+ArcList)
%
%	ArcList contains a list of A-B elements (arcs) where A and B
%       are owl:disjointWith Classes/descriptions.
%	Predicate constructs disjointSet(DescriptionList) of Node
%	descriptions from these arcs such that the sets are the largest
%	possible sets of mutually disjoint Nodes. Blank nodes are used
%	only once, named nodes can be re-used.

owl_disjoint_arcs([A-B|T]) :-
	owl_remove_sym(B-A,T,T1),
	owl_disjoint_nodes([A,B],Set,T1,T2),
	owl_set_descriptions(Set,SetD),
	(   findall(S,(disjointSet(S),subset(SetD,S)),[]),assert(disjointSet(SetD));true),
	owl_disjoint_arcs(T2).

owl_disjoint_arcs([]).


%	owl_set_descriptions(+NodeList,-DescriptionList)
%
%	Get the description for each node in the NodeList. If no
%	description exists, return the Node it self....

owl_set_descriptions([],[]).

owl_set_descriptions([H1|T1],[H2|T2]):-
	owl_description(H1,H2),!,
	owl_set_descriptions(T1,T2).

owl_set_descriptions([H1|T1],[H1|T2]):-
	owl_set_descriptions(T1,T2).



%	owl_disjoint_nodes(+InNodeList,-OutNodeList, +InList,-OutList)
%
%	Calls owl_disjoint_node (see below) for each Node in InNodeList.

owl_disjoint_nodes(SetIn,SetOut,ArcsIn,ArcsOut) :-
	owl_disjoint_node(X,SetIn,ArcsIn,ArcsOut1),!,
	owl_disjoint_nodes([X|SetIn],SetOut,ArcsOut1,ArcsOut).

owl_disjoint_nodes(X,X,Arcs,Arcs).


%	owl_disjoint_node(?Node,-NodeList, +InList,-OutList)
%
%	InList contains a list of A-B elements where A and B
%       are owl:disjointWith Classes/descriptions.
%	The predicate appends Node into NodeList if there is a X in
%	NodeList such that Node-X (or X-Node) exists in InList.
%	In such a case Node-X (or X-Node) are removed from InList
%	(only if either X or Node are blank noded) and the resulted list
%	is OutList.

owl_disjoint_node(_,[],T,T) :- !.
owl_disjoint_node(Node,[H|T],ArcsIn,ArcsOut ):-
	(   member(Node-H,ArcsIn),owl_remove_sym(Node-H,ArcsIn,ArcsOut1);
	    member(H-Node,ArcsIn),owl_remove_sym(H-Node,ArcsIn,ArcsOut1)),
	not(memberchk(Node,[H|T])),
	owl_disjoint_node(Node,T,ArcsOut1,ArcsOut).



%	owl_remove(+Element,+InList,-OutList)
%
%	Outlist is InList with all occurences of Element removed.

owl_remove(_,[],[]) :- !.
owl_remove(H,[H|T],T1) :- !,
	owl_remove(H,T,T1).
owl_remove(H,[A|T],[A|T1]) :-
	owl_remove(H,T,T1).


%	owl_remove_sym(A-B,+InList,-OutList)
%
%	Element is expected to be in the form A-B where A and B
%       are owl:disjointWith Classes/descriptions.
%	The predicate removes from InList all occurences of A-B and B-A
%       in case either A or B are blank (unamed) nodes.
%       The resulted list is OutList.

owl_remove_sym(A-B,In,Out) :-
	(  sub_string(A,0,2,_,'__'); sub_string(B,0,2,_,'__')),
	owl_remove(A-B,In,In1),
	owl_remove(B-A,In1,Out),!.
owl_remove_sym(_,In,In).


% --------------------------------------------------------------------
%                              Properties
%	owl_parse_property
%
%	Any named node defined as having rdf:type any of the OWL
%	defined property types (e.g. Object, Datatype, Functional,
%	etc) is asserted into the database as a property/7 term with all
%	super properties, annotations, range and domain
%	information defined for this Property ID Note that the
%	construction of a property term cannot be done incrementally,
%	i.e. we cannot add ranges, domains or annotations to an existing
%	property.
%
%       property(PropertyId,
%                Deprecated(true/false),
%                AnnotationsList
%	         SuperPropertyList
%                PropertyTypeList
%                DomainList  (DescriptionList)
%                RangeList   (DescriptionList)

owl_parse_property :-
	owl_parse_property_type(PID,PTList),
	owl_deprecated_property(PID,Deprecated),
	not(sub_string(PID,0,2,_,'__')),
	not(property(PID,_,_,_,_,_,_)),
	findall(Xs, use_owl(PID,'rdfs:subPropertyOf',Xs),PID_SuperList),
	findall(Dd, (use_owl(PID,'rdfs:domain',Xd),owl_description(Xd,Dd)), PID_DomainList),
	findall(Dr,(use_owl(PID,'rdfs:range',Xr), owl_description(Xr,Dr)), PID_RangeList),
	findall(CA,(owl_annotation(PID,CA)), AnnotationList),
	assert(property(PID,Deprecated,AnnotationList,PID_SuperList,
			      PTList,PID_DomainList,PID_RangeList)),
	fail.

%       The second clause collects the equivalentProperty axioms.

owl_parse_property:-
	use_owl(X,'owl:equivalentProperty',Y),
	owl_collect_linked_nodes(X,'owl:equivalentProperty',[],List1),
	owl_collect_linked_nodes(Y,'owl:equivalentProperty',List1,List2),
	assert(equivalentSet(List2)),
	fail.

owl_parse_property.


%	owl_parse_property_type(-PID,+[-OT,-F,-IF,-T,-S,iof(-Inv)])
%
%	Returns a PropertyID and the correct property type as a list
%	of atoms describing the property type.

owl_parse_property_type(PID,[OT,F,IF,T,S,iof(Inv)]) :-
	(   use_owl(PID,'rdf:type','owl:DatatypeProperty'), OT = datatypeProperty ; true),
	(   use_owl(PID,'rdf:type','owl:ObjectProperty'), OT = objectProperty ; true),
	(   use_owl(PID,'rdf:type','owl:FunctionalProperty'),F=functional ; true),
	(   use_owl(PID,'rdf:type','owl:InverseFunctionalProperty'),IF=inversefunctional; true),
	(   use_owl(PID,'rdf:type','owl:TransitiveProperty'), T=transitive; true),
	(   use_owl(PID,'rdf:type','owl:SymmetricProperty'), S=symmetric; true),
	(   use_owl(PID,'owl:inverseOf', Inv); true),
	nonvar(PID).

%	owl_annotation(+C,annotation(-APID,-Value)
%
%	For a given name id (C) it returns an annotation construct.
%       APID is either an existing annotation Property, or it is a new
%       one.
%       Predefined annotation properties are rdfs:comment, rdfs:label,
%	rdfs:seeAlso.

owl_annotation(C,annotation(APID,Value)) :-
	annotationProperty(APID),
	use_owl(C,APID,Value).

owl_annotation(C,annotation(APID,Value)) :-
	use_owl(APID,'rdf:type','owl:AnnotationProperty'),
	(   use_owl(APID,'rdf:type','rdf:Property'),! ; true),
	not(sub_string(APID,0,2,_,'__')),
	not(annotationProperty(APID)),
	assert(annotationProperty(APID)),
	use_owl(C,APID,Value).

owl_annotation(C, annotation('rdfs:comment',CA)) :-
  	use_owl(C,'rdfs:comment',CA).

owl_annotation(O, annotation('rdfs:label',OA)) :-
  	use_owl(O,'rdfs:label',OA).

owl_annotation(O, annotation('rdfs:seeAlso',OA)) :-
  	use_owl(O,'rdfs:seeAlso',OA).


%	owl_parse_annotationPropery.
%
%	It creates an annotationProperty term for each occurence of an
%	owl:AnnotationProperty typed ID.
%	Range properies for annotation are not processed yet.

owl_parse_annotationProperty :-
	use_owl(PID, 'rdf:type', 'owl:AnnotationProperty'),
	not(sub_string(PID,0,2,_,'__')),
	not(annotationProperty(PID)),
	assert(annotationProperty(PID)),
	% get all range clauses for annotation but don't do anything at the moment
	findall(Dr, (use_owl(PID,'rdfs:range',Xr),owl_description(Xr,Dr)), _),

	% use_owl(PID, 'rdf:type','rdf:Property'); true,
	fail.

owl_parse_annotationProperty.

% -----------------------------------------------------------------------
%                              Ontology
%      get_ontology.
%
%      Parses owl:Ontology types and creates ontology/2 terms as part
%      of abstract syntax. of core owl_parser, parses the Ontology
%      properties and annotations.

owl_parse_ontology :-
	use_owl(O,'rdf:type','owl:Ontology'),
        findall(OA,(owl_ontology_annotation(O,OA);owl_annotation(O,OA)), OAL),
	assert(ontology(O,OAL)),fail.

owl_parse_ontology.

%      get_ontology_annotation(+P,annotation(-P,-Value)).
%
%      Parses OntologyProperties and ontology-specific annotation
%      properties:
%	  owl:versionInfo, rdfs:isDefineBy, owl:imports,
%	  owl:backwardCompatibleWith, owl:priorVersion.

owl_ontology_annotation(O, annotation(P,O1)) :-
	use_owl(P,'rdf:type','owl:OntologyProperty'),
        use_owl(O,P,O1),
  	use_owl(O1,'rdf:type','owl:Ontology'),!.

owl_ontology_annotation(O, annotation('owl:versionInfo',OA)) :-
  	use_owl(O,'owl:versionInfo',OA).

owl_ontology_annotation(O, annotation('rdfs:isDefinedBy',OA)) :-
  	use_owl(O,'rdfs:isDefinedBy',OA).

owl_ontology_annotation(O, annotation('owl:imports',OA)) :-
  	use_owl(O,'owl:imports',OA).

owl_ontology_annotation(O, annotation('owl:backwardCompatibleWith',OA)) :-
  	use_owl(O,'owl:backwardCompatibleWith',OA).

owl_ontology_annotation(O, annotation('owl:priorVersion',OA)) :-
  	use_owl(O,'owl:priorVersion',OA).


% --------------------------------------------------------------------
%                             Individuals
%
%       owl_parse_named_individuals
%
%	Any named node not defined as an individual is sserted into the
%	database as a individula/5 term with all types, properties and
%	annotations defined with this named individual as a subject.
%	Note that the construction of an individual term cannot
%	be done incrementally, i.e. we cannot add types,
%	properties or annotations to an existing individual.


owl_parse_named_individuals :-
	owl(I,_,_,not_used),
	not(sub_string(I,0,2,_,'__')), not(individual(I,_,_,_)),
	findall(T, (use_owl(I,'rdf:type',T1),owl_description(T1,T)),ITList),
	findall(value(P,V), ((property(P,_,_,_,_,_,_);
			     annotationProperty(P)),use_owl(I,P,V)), IVList),
 	findall(A,(owl_annotation(I,A)), AnnotationList),
	assert(individual(I,AnnotationList,ITList,IVList)),fail.

owl_parse_named_individuals.


%       owl_parse_unnamed_individuals
%
%	Same as above for unnamed individuals.

owl_parse_unnamed_individuals:-
	owl(I,_,_,not_used),not(individual(I,_,_,_)),
	findall(T, (use_owl(I,'rdf:type',T1),owl_description(T1,T)),ITList),
	findall(value(P,V), ((property(P,_,_,_,_,_,_);
			     annotationProperty(P)),use_owl(I,P,V)), IVList),
	findall(A,(owl_annotation(I,A)), AnnotationList),
	assert(individual(I,AnnotationList,ITList,IVList)),fail.

owl_parse_unnamed_individuals.

% --------------------------------------------------------------------
%                           Individual Axioms
%               (AllDifferent, differentFrom, sameAs)


%      owl_parse_individual_axioms/0
%
%      Handles the owl:AllDiferent axiom by asserting a
%      differentIndividuals(List) prolog term where List is a list of
%      different individuals.
%      The second clause of this term handles the differentFrom
%      construct.

owl_parse_individual_axioms :-
	use_owl(X,'rdf:type','owl:AllDifferent'),
	use_owl(X,'owl:distinctMembers',L),
	owl_individual_list(L,L1),
	assert(differentIndividuals(L1)),fail.

owl_parse_individual_axioms :-
	findall(X-Y,use_owl(X,'owl:differentFrom',Y),Arcs),
	owl_different_arcs(Arcs),
	owl_parse_individual_axioms_2.


%      owl_parse_individual_axioms_2.
%
%      Handles the owl:sameAs axiom by asserting a
%      sameIndividuals(List) prolog term where List is a list of
%      sam individuals.

owl_parse_individual_axioms_2 :-
	use_owl(X,'owl:sameAs',Y),
	owl_collect_linked_nodes(X,'owl:sameAs',[],List1),
	owl_collect_linked_nodes(Y,'owl:sameAs',List1,List2),
	assert(sameIndividuals(List2)),
	fail.

owl_parse_individual_axioms_2.


%	owl_different_arcs(+ArcList)
%
%	ArcList contains a list of A-B elements (arcs) where A and B
%       are owl:differentFrom Individuals.
%	Predicate constructs disjointIndividuals(IndividualList) of
%	Nodes from these arcs such that the sets are the
%	largest possible sets of mutually different Nodes. Blank nodes
%	are used only once, named nodes can be re-used.

owl_different_arcs([A-B|T]) :-
	owl_remove_sym(B-A,T,T1),
	owl_different_nodes([A,B],Set,T1,T2),
	(   findall(S,(differentIndividuals(S),subset(Set,S)),[]),
	    assert(differentIndividuals(Set));true),
	owl_different_arcs(T2).

owl_different_arcs([]).


%	owl_different_nodes(+InNodeList,-OutNodeList, +InList,-OutList)
%
%	Calls owl_different_node (see below) for each Node in
%	InNodeList.

owl_different_nodes(SetIn,SetOut,ArcsIn,ArcsOut) :-
	owl_different_node(X,SetIn,ArcsIn,ArcsOut1),!,
	owl_different_nodes([X|SetIn],SetOut,ArcsOut1,ArcsOut).

owl_different_nodes(X,X,Arcs,Arcs).


%	owl_different_node(?Node,-NodeList, +InList,-OutList)
%
%	InList contains a list of A-B elements where A and B
%       are owl:differentFrom Individuals.
%	The predicate appends Node into NodeList if there is a X in
%	NodeList such that Node-X (or X-Node) exists in InList.
%	In such a case Node-X (or X-Node) are removed from InList
%	(only if either X or Node are blank noded) and the resulted list
%	is OutList.

owl_different_node(_,[],T,T) :- !.
owl_different_node(Node,[H|T],ArcsIn,ArcsOut ):-
	(   member(Node-H,ArcsIn),owl_remove(Node-H,ArcsIn,ArcsOut1);
	    member(H-Node,ArcsIn),owl_remove(H-Node,ArcsIn,ArcsOut1)),
	not(memberchk(Node,[H|T])),
	owl_different_node(Node,T,ArcsOut1,ArcsOut).













