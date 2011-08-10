

:- rdf_meta
    owlapi_subClassOf(r,r).



:- assert(owl_manager(fail)).
owlapi_manager(OWLManager) :-
    owl_manager(fail),
    jpl_call('org.semanticweb.owlapi.apibinding.OWLManager', createOWLOntologyManager, [], OWLManager),
    retract(owl_manager(fail)),
    assert(owl_manager(OWLManager)),!.

owlapi_manager(OWLManager) :-
    owl_manager(OWLManager).




owlapi_load_ontology(FileName, OWLManager, OWLOntology) :-
    jpl_new('java.io.File', [FileName], OntFile),
    jpl_call(OWLManager, loadOntologyFromOntologyDocument, [OntFile], OWLOntology),
    jpl_call(OWLOntology, getOntologyID, [], OntoID),
    jpl_call(OntoID, toString, [], OntoIDstr),
    format('loaded ontology ~w\n', OntoIDstr).




owlapi_new_ontology(Base, OWLManager, OWLOntology) :-
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Base], BaseIRI),
    jpl_call(OWLManager, createOntology, [BaseIRI], OWLOntology).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


:- assert(owl_datafact(fail)).
owlapi_datafactory(OWLManager, DataFactory) :-
    owl_datafact(fail),
    jpl_call(OWLManager, getOWLDataFactory, [], DataFactory),
    retract(owl_datafact(fail)),
    assert(owl_datafact(DataFactory)),!.

owlapi_datafactory(_, DataFactory) :-
    owl_datafact(DataFactory).


owlapi_get_class(ClassName, DataFactory, JavaClass) :-
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [ClassName], IRI),
    jpl_call(DataFactory, getOWLClass, [IRI], JavaClass).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


:- assert(owl_hermit(fail)).
owlapi_reasoner_hermit(OWLOntology, Reasoner) :-
    owl_hermit(fail),
    jpl_new('org.semanticweb.HermiT.Reasoner', [OWLOntology], Reasoner),
    retract(owl_hermit(fail)),
    assert(owl_hermit(Reasoner)),!.

owlapi_reasoner_hermit(_, Reasoner) :-
    owl_hermit(Reasoner).


% bad reasoner:
% owlapi_reasoner_pellet(OWLOntology, Reasoner) :-
%     jpl_new('com.clarkparsia.pellet.owlapiv3.PelletReasonerFactory', [], ReasonerFactory),
%     jpl_new('org.semanticweb.owlapi.reasoner.ConsoleProgressMonitor', [], ProgMon),
%     jpl_new('org.semanticweb.owlapi.reasoner.SimpleConfiguration', [ProgMon], Config),
%     jpl_call(ReasonerFactory, createReasoner, [OWLOntology, Config], Reasoner).


owlapi_subClassOf(Sub, Super) :-
    owlapi_manager(OWLManager),
    owlapi_datafactory(OWLManager, DataFactory),

    % TODO: get ontology from Prolog
    owlapi_reasoner_hermit(_OWLOntology, Reasoner),
    owlapi_reasoner_subclasses(Reasoner, DataFactory, Super, Sub).

owlapi_reasoner_subclasses(Reasoner, DataFactory, Super, Sub) :-
    owlapi_get_class(Super, DataFactory, SuperCl),
    jpl_call(Reasoner, getSubClasses, [SuperCl, @false], Subs),
    owlapi_nodeset_to_list(Subs, SubList),
    member(SubInd, SubList),
    jpl_call(SubInd, toStringID, [], Sub).



owlapi_nodeset_to_list(NodeSet, List) :-
    jpl_call(NodeSet, getFlattened, [], NodeSetFlat),
    jpl_call(NodeSetFlat, toArray, [], NodeSetArray),
    jpl_array_to_list(NodeSetArray, List).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% methods for copying knowledge to the OWLAPI buffer



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% sub-class axioms

add_to_ontology(OWLManager, DataFactory, OWLOntology, Sub) :-

    % read information from the Prolog DB
    rdf(Sub, rdfs:'subClassOf', Super),


    % determine IRIs
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Sub],   SubIRI),
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Super], SuperIRI),

    % create OWLClass objects
    jpl_call(DataFactory, getOWLClass, [SubIRI],   SubClass),
    jpl_call(DataFactory, getOWLClass, [SuperIRI], SuperClass),

    % assert Sub as subClassOf Super
    jpl_call(DataFactory, getOWLSubClassOfAxiom, [SubClass, SuperClass], Ax),

    % commit to ontology
    jpl_call(OWLManager, addAxiom, [OWLOntology, Ax], _).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Disjointness

% AllDisjoint
add_to_ontology(OWLManager, DataFactory, OWLOntology, List) :-

    rdf_has(AllDis, rdf:type, owl:'AllDisjointClasses'),
    rdf_has(AllDis, owl:'members', Set),
    rdfs_list_to_prolog_list(Set, List),

    url_list_to_classexps(DataFactory, List, CExps),
    list_to_set(CExps, CSet),

    jpl_call(DataFactory, getOWLDisjointClassesAxiom, [CSet], DisjointAx),
    jpl_call(OWLManager, addAxiom, [OWLOntology, DisjointAx], _).


% direct assertion for class
add_to_ontology(OWLManager, DataFactory, OWLOntology, A) :-

    rdf_has(A, owl:disjointWith, B),

    url_list_to_classexps(DataFactory, [A,B], CExps),
    list_to_set(CExps, CSet),

    jpl_call(DataFactory, getOWLDisjointClassesAxiom, [CSet], DisjointAx),
    jpl_call(OWLManager, addAxiom, [OWLOntology, DisjointAx], _).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Restrictions


% % % % % % % % % % % % % % % % % % % %
% subClassOf, equivalentClass


% subClassOf
add_to_ontology(OWLManager, DataFactory, OWLOntology, Restr) :-

    % assert class as subClassOf restriction
    rdf_has(Cl, rdfs:subClassOf, Restr),

    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Cl], ClIRI),
    jpl_call(DataFactory, getOWLClass, [ClIRI],  ClClass),

    create_restriction_classexp(DataFactory, Restr, RestrClass),
    jpl_call(DataFactory, getOWLSubClassOfAxiom, [ClClass, RestrClass], Ax),

    % commit to ontology
    jpl_call(OWLManager, addAxiom, [OWLOntology, Ax], _).


% equivalentClass
add_to_ontology(OWLManager, DataFactory, OWLOntology, Restr) :-

    rdf_has(Cl, owl:equivalentClass, Restr),

    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Cl], ClIRI),
    jpl_call(DataFactory, getOWLClass, [ClIRI],  ClClass),

    create_restriction_classexp(DataFactory, Restr, RestrClass),
    jpl_call(DataFactory, getOWLEquivalentClassesAxiom, [ClClass, RestrClass], Ax),

    % commit to ontology
    jpl_call(OWLManager, addAxiom, [OWLOntology, Ax], _).






% % % % % % % % % % % % % % % % % % % %
% unionOf/intersection of

% all of the following predicates get an OWL class expression as input (IRI)
% and create an OWLClassExpression in the OWLAPI format


% intersectionOf
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Class'),
    rdf_has(In, owl:'intersectionOf', IntRDFList),

    rdfs_list_to_prolog_list(IntRDFList, IntList),

    % recursively create the class expressions that are being intersected
    findall(Intersecting, (member(I, IntList), create_restriction_classexp(DataFactory, I, Intersecting)), IntersectionOf),
    list_to_set(IntersectionOf, IntersectionOfSet),

    % create intersection object
    jpl_call(DataFactory, getOWLObjectIntersectionOf, [IntersectionOfSet], Out).


% unionOf
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Class'),
    rdf_has(In, owl:'unionOf', UnionRDFList),

    rdfs_list_to_prolog_list(UnionRDFList, UnionList),

    % recursively create the class expressions that are being uni-fied
    findall(Union, (member(U, UnionList), create_restriction_classexp(DataFactory, U, Union)), UnionOf),
    list_to_set(UnionOf, UnionOfSet),

    % create union object
    jpl_call(DataFactory, getOWLObjectUnionOf, [UnionOfSet], Out).



% normal named class
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Class'),

    \+ rdf_has(In, owl:'unionOf', _),
    \+ rdf_has(In, owl:'intersectionOf', _),

    jpl_call('org.semanticweb.owlapi.model.IRI', create, [In], ClIRI),
    jpl_call(DataFactory, getOWLClass, [ClIRI],  Out).





% % % % % % % % % % % % % % % % % % % %
% create the different types of restrictions


% someValuesFrom restrictions: Object properties
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Restriction'),
    rdf_has(In, owl:'onProperty', Prop),
    rdf_has(In, owl:'someValuesFrom', Val),

    rdf_has(Prop, rdf:type, owl:'ObjectProperty'),

    % determine IRIs
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Prop],  PropIRI),
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Val],   ValIRI),

    % create restriction object
    jpl_call(DataFactory, getOWLObjectProperty,       [PropIRI], PropProp),
    jpl_call(DataFactory, getOWLClass,                [ValIRI],  ValClass),
    jpl_call(DataFactory, getOWLObjectSomeValuesFrom, [PropProp, ValClass], Out).


% allValuesFrom restrictions: Object properties
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Restriction'),
    rdf_has(In, owl:'onProperty', Prop),
    rdf_has(In, owl:'allValuesFrom', Val),

    rdf_has(Prop, rdf:type, owl:'ObjectProperty'),

    % determine IRIs
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Prop],  PropIRI),
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Val],   ValIRI),

    % create restriction object
    jpl_call(DataFactory, getOWLObjectProperty,      [PropIRI], PropProp),
    jpl_call(DataFactory, getOWLClass,               [ValIRI],  ValClass),
    jpl_call(DataFactory, getOWLObjectAllValuesFrom, [PropProp, ValClass], Out).







% someValuesFrom restrictions: Datatype properties
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Restriction'),
    rdf_has(In, owl:'onProperty', Prop),
    rdf_has(In, owl:'someValuesFrom', Val),

    rdf_has(Prop, rdf:type, owl:'DatatypeProperty'),

    % determine IRIs
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Prop],  PropIRI),
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Val],   ValIRI),

    % create restriction object
    jpl_call(DataFactory, getOWLDataProperty,       [PropIRI], PropProp),
    jpl_call(DataFactory, getOWLClass,              [ValIRI],  ValClass),
    jpl_call(DataFactory, getOWLDataSomeValuesFrom, [PropProp, ValClass], Out).


% allValuesFrom restrictions: Datatype properties
create_restriction_classexp(DataFactory, In, Out) :-

    % read information from the Prolog DB
    rdf_has(In, rdf:type, owl:'Restriction'),
    rdf_has(In, owl:'onProperty', Prop),
    rdf_has(In, owl:'allValuesFrom', Val),

    rdf_has(Prop, rdf:type, owl:'DatatypeProperty'),

    % determine IRIs
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Prop],  PropIRI),
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [Val],   ValIRI),

    % create restriction object
    jpl_call(DataFactory, getOWLDataProperty,      [PropIRI], PropProp),
    jpl_call(DataFactory, getOWLClass,             [ValIRI],  ValClass),
    jpl_call(DataFactory, getOWLDataAllValuesFrom, [PropProp, ValClass], Out).


% TODO: cardinality restrictions

% equivalent for minCardinality, maxCardinality,
% OWLDataExactCardinality getOWLDataExactCardinality(int cardinality,
%                                                    OWLDataPropertyExpression property,
%                                                    OWLDataRange dataRange)
% OWLDataExactCardinality getOWLDataExactCardinality(int cardinality,
%                                                    OWLDataPropertyExpression property)







% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Utils



url_list_to_classexps(_, [], []).
url_list_to_classexps(DataFactory, [U|Us], [UClass|Cs]) :-
    jpl_call('org.semanticweb.owlapi.model.IRI', create, [U], UIRI),
    jpl_call(DataFactory, getOWLClass, [UIRI], UClass),
    url_list_to_classexps(DataFactory, Us, Cs).

list_to_set(Values, Set) :-
    jpl_new('java.util.HashSet', [], Set),
    list_to_set_1(Values, Set).

list_to_set_1([], _).
list_to_set_1([V|Vs], Set) :-
    jpl_call(Set, 'add', [V], _),
    list_to_set_1(Vs, Set).

