:- module(semweb,
    [ sw_triple(r,t,t),            % ?Subject, +Predicate, ?Object
      sw_triple(r,t,t,t),          % ?Subject, +Predicate, ?Object, +Context
      sw_instance_of(r,t),         % ?Resource, ?Class
      sw_instance_of(r,t,t),       % ?Resource, ?Class, +Context
      sw_subclass_of(t,t),         % ?SubClass, ?Class
      sw_subproperty_of(r,r),      % ?SubProperty, ?Property

      sw_class_expr(r,t),          % +Class, ?Expr
      sw_instance_of_expr(r,t),    % ?Resource, ?Expr
      sw_subclass_of_expr(t,t),    % ?SubClass, ?Expr

      sw_assert_triple(r,t,t),     % +Subject, +Predicate, +Object
      sw_assert_triple(r,t,t,r),   % +Subject, +Predicate, +Object, +Graph
      sw_assert_triple(r,t,t,r,t), % +Subject, +Predicate, +Object, +Graph, +Scope
      sw_assert_type(r,t),         % +Resource, +Class
      sw_assert_type(r,t,r),       % +Resource, +Class, +Graph
      sw_assert_type(r,t,r,t),     % +Resource, +Class, +Graph, +Scope

      sw_url/4,                    % +URL, ?ResolvedURL, ?OntologyGraph, ?OntologyVersion
      sw_url_graph/2,              % +URL, ?OntologyGraph
      sw_url_version/2,            % +URL, ?OntologyVersion
      sw_register_prefix/2,
      sw_url_register_ns/2,

      sw_graph_include/2,          % +Graph, +IncludedGraph
      sw_graph_includes/2,         % ?Graph, ?IncludedGraph
      sw_unload_graph/1,           % +Graph
      sw_set_default_graph/1,      % +Graph
      sw_default_graph/1,          % ?Graph
      sw_current_graph/2,

      sw_literal_compare/3,     % +Operator, +Literal1, +Literal2
      sw_literal_max/3,         % +Literal1, +Literal2, ?Max
      sw_literal_min/3,         % +Literal1, +Literal2, ?Min

      sw_resource_frequency/2,  % ?Resource, ?Frequency
      sw_class_frequency/2,     % ?Cls, ?Frequency
      sw_property_frequency/2,  % ?Property, ?Frequency

      sw_origin_any/1,          % ?Origin
      sw_origin_system/1, 		% ?Origin
      sw_origin_session/1, 		% ?Origin
      sw_origin_user/1, 		% ?Origin
      sw_origin_reasoner/1, 	% ?Origin
      sw_origin_test/1, 		% ?Origin

      load_rdf_xml/2               % +URL, +ParentGraph
    ]).
/** <module> Extensions around the semweb modules of Prolog.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('logging')).
:- use_module(library('rdf'),
		[ load_rdf/3 ]).
:- use_module(library('semweb/rdf_db'), 
		[ rdf_equal/2,
		  rdf_load/2,
		  rdf_assert/4,
		  rdf_has/3,
		  rdf_has/4,
		  rdf_equal/2,
		  rdf_unload_graph/1,
		  rdf_is_literal/1,
		  rdf_resource/1,
		  rdf_subject/1,
		  rdf_literal_value/2,
		  rdf_register_prefix/3,
		  rdf/3,
		  rdf/4 ]).
:- use_module(library('semweb/rdf11'),
		[ rdf_list/2 ]).
:- use_module(library('semweb/rdfs'),
		[ rdfs_individual_of/2,
		  rdfs_subclass_of/2,
		  rdfs_subproperty_of/2 ]).
:- use_module(library('http/http_open'),
		[ http_open/3 ]).
:- use_module(library('ext/xsd'),
		[ xsd_data_basetype/2 ]).
:- use_module(library('scope'),
		[ query_scope_now/1 ]).
:- use_module(library(dcg/basics)).

%%
annotation_property('http://www.w3.org/2000/01/rdf-schema#comment').
annotation_property('http://www.w3.org/2000/01/rdf-schema#seeAlso').
annotation_property('http://www.w3.org/2000/01/rdf-schema#label').
annotation_property('http://www.w3.org/2002/07/owl#versionInfo').

% TODO: support temporally scoped predicates
% TODO: support fuzzy predicates
% TODO: set predicate properties, rdf_db supports some
%   - rdf_predicate_property(?Predicate, ?Property)
%       - symmetric(Bool), inverse_of(Inverse), transitive(Bool)

		 /*******************************
		  *          QUERYING           *
		  *******************************/

%% sw_triple(?Subject,+Predicate,?Object) is nondet.
%
% Same as sw_triple/4 but only includes triples that are
% currently true.
%
sw_triple(Subject, Predicate, Object) :-
	query_scope_now(QScope),
    sw_triple(Subject, Predicate, Object, [query_scope(QScope)]).

%% sw_triple(?Subject,+Predicate,?Object,+Conext) is nondet.
%
% Succeeds if the triple (Subject, Predicate, Object) is true
% exploiting the rdfs:subPropertyOf predicate.
%
sw_triple(Subject, Predicate, Object, _Context) :-
    atom(Object),!,
    % TODO: better check if it is datatype property, and query literal in this case?
    rdf_has(Subject, Predicate, Object).

sw_triple(Subject, Predicate, Object, _Context) :-
    var(Object),!,
    rdf_has(Subject, Predicate, Value),
    % convert XSD atom value into native type
    once( atom(Value) -> Object=Value
    	; rdf_literal_value(Value,Object)
    	% rdf_literal_value does not handle boolean.
    	% So in case it fails, we just strip the type
    	; Value=literal(type(_,Object))
    	; Value=literal(Object)
    ).

sw_triple(Subject, Predicate, Object, _Context) :-
    compound(Object),!,
    % typed literals are represented as: `Type(Value)`
    Object =.. [Type,Value],
    % get XSD IRI
    xsd_data_basetype(XSDType, Type),
    % typed triple lookup
    LiteralValue = literal(type(XSDType, Value)),
    rdf_has(Subject, Predicate, LiteralValue).

sw_triple(Subject, Predicate, Number, _Context) :-
    number(Number),!,
    atom_number(ValueAtom, Number),
    LiteralValue = literal(type(_XSDType, ValueAtom)),
    rdf_has(Subject, Predicate, LiteralValue).

sw_triple(Subject, Predicate, Object, _Context) :-
    throw(error(type_error(resource, Object),
                sw_triple(Subject,Predicate,Object))).

%% sw_literal_compare(+Operator, +Literal1, +Literal2) is nondet.
%
% Comparison of typed literals using the given operator.
% Operator can be one of: ==, !=, >, <, >=, <=, eq, gt, lt, ge, le.
%
sw_literal_compare(_Operator, Literal1, Literal2) :-
	(\+ ground(Literal1) ; \+ ground(Literal2)),
	% pass through if one of the literals is not ground
	!.
sw_literal_compare(Operator, Literal1, Literal2) :-
	% convert literals to native types
	(rdf_literal_value(Literal1, Value1) -> true ; Value1=Literal1),
	(rdf_literal_value(Literal2, Value2) -> true ; Value2=Literal2),
	% compare values
	once(sw_literal_compare_(Operator, Value1, Value2)).

sw_literal_compare_('==', Value1, Value2) :- Value1 == Value2.
sw_literal_compare_('!=', Value1, Value2) :- Value1 \= Value2.
sw_literal_compare_('>', Value1, Value2) :- Value1 > Value2.
sw_literal_compare_('<', Value1, Value2) :- Value1 < Value2.
sw_literal_compare_('>=', Value1, Value2) :- Value1 >= Value2.
sw_literal_compare_('<=', Value1, Value2) :- Value1 =< Value2.
sw_literal_compare_('=<', Value1, Value2) :- Value1 =< Value2.
sw_literal_compare_('eq', Value1, Value2) :- Value1 == Value2.
sw_literal_compare_('gt', Value1, Value2) :- Value1 > Value2.
sw_literal_compare_('lt', Value1, Value2) :- Value1 < Value2.
sw_literal_compare_('ge', Value1, Value2) :- Value1 >= Value2.
sw_literal_compare_('le', Value1, Value2) :- Value1 =< Value2.

%% sw_literal_min(+Literal1, +Literal2, ?Min) is nondet.
%
% Bind the miminum of two literals to Min.
% A unbound literal is assumed to be the maximum.
%
sw_literal_min(Literal1, Min, Min) :- \+ ground(Literal1), !.
sw_literal_min(Min, Literal2, Min) :- \+ ground(Literal2), !.
sw_literal_min(Literal1, Literal2, Min) :-
	(rdf_literal_value(Literal1, Value1) -> true ; Value1=Literal1),
	(rdf_literal_value(Literal2, Value2) -> true ; Value2=Literal2),
	(   Value1 < Value2
	->  Min=Literal1
	;   Min=Literal2
	).

%% sw_literal_max(+Literal1, +Literal2, ?Max) is nondet.
%
% Bind the maximum of two literals to Max.
% A unbound literal is assumed to be the minimum.
%
sw_literal_max(Literal1, Max, Max) :- \+ ground(Literal1), !.
sw_literal_max(Max, Literal2, Max) :- \+ ground(Literal2), !.
sw_literal_max(Literal1, Literal2, Max) :-
	(rdf_literal_value(Literal1, Value1) -> true ; Value1=Literal1),
	(rdf_literal_value(Literal2, Value2) -> true ; Value2=Literal2),
	(   Value1 > Value2
	->  Max=Literal1
	;   Max=Literal2
	).

%% sw_resource_frequency(?Resource, -Frequency) is nondet.
%
sw_resource_frequency(Resource, Frequency) :-
	sw_class_frequency(Resource, Frequency) ;
	sw_property_frequency(Resource, Frequency).

%% sw_class_frequency(?Cls, -Frequency) is nondet.
%
sw_class_frequency(Cls, Frequency) :-
	rdf_has(Cls, rdf:type, owl:'Class'),
	findall([S,P], rdf_has(S, P, Cls), Xs),
	length(Xs, Frequency).

%% sw_property_frequency(?Property, -Frequency) is nondet.
%
sw_property_frequency(Property, Frequency) :-
	( rdf_has(Property, rdf:type, owl:'ObjectProperty')
	; rdf_has(Property, rdf:type, owl:'DatatypeProperty')
	),
	findall([S,O], rdf_has(S, Property, O), Xs),
	length(Xs, Frequency).

%%
%post_graph(Subject, RealPredicate, Object) :-
%    current_reasoner_module(Reasoner),
%    rdf(Subject, RealPredicate, Object, Graph:_),
%    sw_current_graph(Reasoner, Graph),
%    !.

%% sw_instance_of(?Resource, ?Class) is nondet.
%
% Same as sw_instance_of/2 but checks if Resource is a current
% instance of Class.
%
sw_instance_of(Resource, Class) :-
	query_scope_now(QScope),
    sw_instance_of(Resource, Class, [query_scope(QScope)]).

%% sw_instance_of(?Resource, ?Class, +Context) is nondet.
%
% True if Resource is an individual of Class.
% This implies Resource has an rdf:type property that refers to Class or a sub-class thereof.
%
sw_instance_of(Resource, Class, _Context) :-
    atom(Class),!,
    rdfs_individual_of(Resource,Class).

sw_instance_of(Resource, Expr, _Context) :-
    compound(Expr),!,
    sw_instance_of_expr(Resource,Expr).

sw_instance_of(Resource, Cls, _Context) :-
    throw(error(type_error(resource, Cls),
                sw_instance_of(Resource,Cls))).

%% sw_subclass_of(?SubClass, ?Class) is nondet.
%
% True if SubClass is equal to Class or Class can be reached from
% SubClass following the rdfs:subClassOf relation.
%
sw_subclass_of(SubClass,Class) :-
    (atom(Class);var(Class)),!,
    rdfs_subclass_of(SubClass,Class).

sw_subclass_of(SubClass,Class) :-
    throw(error(type_error(resource, Class),
                sw_subclass_of(SubClass,Class))).

%% sw_subproperty_of(?SubProperty, ?Property) is nondet.
%
% True if SubProperty is equal to Property or Property can be reached
% from SubProperty following the rdfs:subPropertyOf relation.
%
sw_subproperty_of(SubProperty,Property) :-
    (atom(Property);var(Property)),!,
    rdfs_subproperty_of(SubProperty,Property).

sw_subproperty_of(SubProperty,Property) :-
    throw(error(type_error(resource, Property),
                sw_subproperty_of(SubProperty,Property))).

		 /*******************************
		  *       CLASS EXPRESSIONS     *
		  *******************************/

%% sw_class_expr(+Class, ?Expr) is semidet.
%
% True if Expr is the class expression of Class.
% Expr has one of the following forms:
%
%    * only(Property,Class)
%    * some(Property,Class)
%    * value(Property,Value)
%    * min(Property,Min,Class)
%    * max(Property,Max,Class)
%    * exactly(Property,Count,Class)
%    * union_of(Classes)
%    * intersection_of(Classes)
%    * complement_of(Class)
%    * one_of(Individuals)
%
sw_class_expr(Class, Expr) :-
    \+ ground(Class),!,
    throw(error(instantiation_error(Class), sw_class_expr(Class, Expr))).

sw_class_expr(Class, Expr) :-
    sw_restriction_expr(Class,Expr),!.

sw_class_expr(Class, union_of(ListOfExpr)) :-
    rdf(Class, owl:unionOf, ListIRI),
    class_list_expr(ListIRI, ListOfExpr),!.

sw_class_expr(Class, intersection_of(ListOfExpr)) :-
    rdf(Class, owl:intersectionOf, ListIRI),
    class_list_expr(ListIRI, ListOfExpr),!.

sw_class_expr(Class, one_of(ListOfExpr)) :-
    rdf(Class, owl:oneOf, ListIRI),
    value_list_expr(ListIRI, ListOfExpr),!.

sw_class_expr(Class1, complement_of(Expr2)) :-
    rdf(Class1, owl:complementOf, Class2),
    sw_class_expr(Class2, Expr2),!.

sw_class_expr(Class, class(Class)) :-
    rdf_resource(Class),!.

sw_class_expr(Class, Expr) :-
    throw(error(type_error(class, Class), sw_class_expr(Class,Expr))).

%%
sw_restriction_expr(R, Expr) :-
    restriction_expr0(R, P, Expr),
	rdf(R,owl:onProperty,P).

restriction_expr0(R, P, Expr) :-
	rdf(R,owl:onClass,Cls),
	sw_class_expr(Cls,ClsExpr),
	restriction_expr2(R, P, ClsExpr, Expr).
restriction_expr0(R, P, Expr) :- restriction_expr1(R, P, Expr).

restriction_expr1(R, P, only(P,O))    :- rdf(R,owl:allValuesFrom,Cls),  sw_class_expr(Cls,O).
restriction_expr1(R, P, some(P,O))    :- rdf(R,owl:someValuesFrom,Cls), sw_class_expr(Cls,O).
restriction_expr1(R, P, value(P,O))   :- rdf(R,owl:hasValue,Value),     value_expr(Value,O).
restriction_expr1(R, P, min(P,C))     :- rdf(R,owl:minCardinality,Lit), rdf_literal_value(Lit,C).
restriction_expr1(R, P, max(P,C))     :- rdf(R,owl:maxCardinality,Lit), rdf_literal_value(Lit,C).
restriction_expr1(R, P, exactly(P,C)) :- rdf(R,owl:cardinality,Lit),    rdf_literal_value(Lit,C).

restriction_expr2(R, P, Cls, min(P,C,Cls))     :- rdf(R,owl:minQualifiedCardinality,Lit), rdf_literal_value(Lit,C).
restriction_expr2(R, P, Cls, max(P,C,Cls))     :- rdf(R,owl:maxQualifiedCardinality,Lit), rdf_literal_value(Lit,C).
restriction_expr2(R, P, Cls, exactly(P,C,Cls)) :- rdf(R,owl:qualifiedCardinality,Lit),    rdf_literal_value(Lit,C).

class_list_expr(ListIRI, ListOfExpr) :-
	rdf_list(ListIRI, ListOfIRIs),
	findall(MemberExpr,
	    (   member(MemberIRI,ListOfIRIs),
	        sw_class_expr(MemberIRI, MemberExpr)
	    ), ListOfExpr).

value_list_expr(ListIRI, ListOfExpr) :-
	rdf_list(ListIRI, Values),
	findall(ValueExpr,
	    (   member(Value,Values),
	        value_expr(Value, ValueExpr)
	    ), ListOfExpr).

value_expr(Value, Expr) :-
    (   rdf_is_literal(Value) -> Expr=Value
    ;   Expr=individual(Value)
    ).

%% sw_instance_of_expr(?Resource, ?Expr) is nondet.
%
% True if it can be deduced that Resource is an instance of Expr.
% It is expected that the ontology is classified apriori --
% there is no interaction with an OWL reasoner when this
% predicate is evaluated.
%
sw_instance_of_expr(Resource, Expr) :-
    (   ground([Resource,Expr])
    ->  once(sw_instance_of_expr1(Resource, Expr))
    ;   sw_instance_of_expr1(Resource, Expr)
    ).

sw_instance_of_expr1(Resource, class(Cls)) :-
    !,
    rdfs_individual_of(Resource, Cls).

sw_instance_of_expr1(Resource, one_of(ExprList)) :-
    !,
    % TODO: take into account sameAs
    member(individual(Resource), ExprList).

sw_instance_of_expr1(Resource, union_of(ExprList)) :-
    !,
    member(Expr, ExprList),
    sw_instance_of_expr1(Resource, Expr).

sw_instance_of_expr1(Resource, intersection_of([Expr])) :-
    !,
    sw_instance_of_expr1(Resource, Expr).
sw_instance_of_expr1(Resource, intersection_of([Expr0|RestExpr])) :-
    !,
    sw_instance_of_expr1(Resource, Expr0),
    forall(member(ExprN, RestExpr), sw_instance_of_expr1(Resource, ExprN)).

sw_instance_of_expr1(Resource, Expr) :-
    atom(Resource),!, % generate all instance-of expressions of Resource
    rdf(Resource, rdf:type, Class),
    sw_subclass_of_expr(Class, Expr).

sw_instance_of_expr1(Resource, Expr) :-
    !, % var(Resource)
    % NOTE: value/2, only/2, some/2, min/3, max/3, exactly/3, min/2, max/2, exactly/2
    %       are not handled here. one could iterate _all_ individuals _and_ their
    %       types. Then for each type check if it is a subclass of Expr
    %       (basically above clause without type check)
    %       but this might be very inefficient.
    throw(error(instantiation_error(Resource), sw_instance_of_expr1(Resource, Expr))).

%% sw_subclass_of_expr(?Class, ?Expr) is nondet.
%
% If var(Expr), then all class expressions of direct superclasses
% are generated.
%
sw_subclass_of_expr(Class, Expr) :-
    atom(Class),!,
    subclass_expr1(Class, Expr).

sw_subclass_of_expr(Class, Expr) :-
    \+ ground(Class),!,
    throw(error(instantiation_error(Class), sw_subclass_of_expr(Class, Expr))).

%%
subclass_expr1(Class, Expr) :-
    var(Expr),!, % generate all subclass of expressions of Class
    rdf(Class, rdfs:subClassOf, Super),
    sw_class_expr(Super, Expr).

subclass_expr1(Class, Expr) :-
    sw_class_expr(Class, SubExpr),
    subclass_expr2(SubExpr, Expr).

%%
subclass_expr2(Expr,Expr) :- !.

subclass_expr2(Expr1, union_of(Classes)) :-
    !,
    once((member(Expr2, Classes), subclass_expr2(Expr1, Expr2))).
subclass_expr2(Expr1, intersection_of(Classes)) :-
    !,
    forall(member(Expr2, Classes), subclass_expr2(Expr1, Expr2)).

% map some/2 to min/3
subclass_expr2(some(Property1,Expr1), Expr2) :-
    !,
    subclass_expr2(min(1,Property1,Expr1), Expr2).
subclass_expr2(Expr1, some(Property2,Expr2)) :-
    !,
    subclass_expr2(Expr1, min(1,Property2,Expr2)).

% handle class/1 as first argument
subclass_expr2(class(Class1), class(Class2)) :-
    !,
    rdfs_subclass_of(Class1, Class2).
subclass_expr2(class(Class), Expr2) :-
    !,
    rdf(Class, rdfs:subClassOf, Super),
    sw_class_expr(Super, Expr1),
    subclass_expr2(Expr1, Expr2).

% handle value/2 as first argument
subclass_expr2(value(P1, Value), value(P2, Value)) :-
    !,
    rdfs_subproperty_of(P1, P2).
subclass_expr2(value(P1, individual(Value)), min(1, P2, Expr2)) :-
    !,
    rdfs_subproperty_of(P1, P2),
    sw_instance_of_expr(Value, Expr2).

% handle complement_of/1 as first argument
subclass_expr2(complement_of(Expr1), complement_of(Expr2)) :-
    !, % e.g. `not(Food) subclassOf not(Rice)` follows from `Rice subclassOf Food`
    subclass_expr2(Expr2, Expr1).
%subclass_expr2(Expr1, complement_of(Expr2)) :-
%    \+ sw_satisfiable_expr(intersection_of(Expr1, Expr2)).

% handle union_of/1 as first argument
subclass_expr2(union_of(Classes), Expr2) :-
    !, % e.g. `union([Tea,Cola]) subclassOf Drink` because Tea and Cola both are subclassOf Drink
    forall(member(Expr1, Classes), subclass_expr2(Expr1, Expr2)).

% handle universal restriction in the first argument
subclass_expr2(only(Property1,Expr1), only(Property2,Expr2)) :-
    !,
    rdfs_subproperty_of(Property1, Property2),
    subclass_expr2(Expr1, Expr2).

% handle cardinality restriction in the first argument
subclass_expr2(min(Card1,Property1,Expr1), min(Card2,Property2,Expr2)) :-
    !,
    Card1 >= Card2,
    rdfs_subproperty_of(Property1, Property2),
    subclass_expr2(Expr1, Expr2).
subclass_expr2(min(Card1,Property1), min(Card2,Property2)) :-
    !,
    Card1 >= Card2,
    rdfs_subproperty_of(Property1, Property2).

		 /*******************************
		  *          ASSERTIONS         *
		  *******************************/

%% sw_assert_triple(+Subject, +Predicate, +Object) is det.
%
% Same as sw_assert_triple/5 but asserts a universal (unscoped) fact
% into the default graph (see sw_default_graph/1).
%
sw_assert_triple(Subject, Predicate, Object) :-
    sw_default_graph(Graph),
    sw_assert_triple(Subject, Predicate, Object, Graph).

%% sw_assert_triple(+Subject, +Predicate, +Object, +Graph) is det.
%
% Same as sw_assert_triple/5 but asserts a universal (unscoped) fact.
%
sw_assert_triple(Subject, Predicate, Object, Graph) :-
    %sw_universal_scope(Scope),
    sw_assert_triple(Subject, Predicate, Object, Graph, _).

%% sw_assert_triple(+Subject, +Predicate, +Object, +Graph, +Scope) is det.
%
% Assert a new triple into the database.
%
sw_assert_triple(Subject, Predicate, Object, Graph, _Scope) :-
    atom(Subject), atom(Predicate), ground(Object),!,
    % todo: support scoped assertions
    (  rdfs_individual_of(Predicate,owl:'DatatypeProperty')
    -> sw_assert_dataproperty(Subject, Predicate, Object, Graph)
    ;  sw_assert_objectproperty(Subject, Predicate, Object, Graph)
    ).

%%
sw_assert_objectproperty(Subject, Predicate, Object, Graph) :-
    atom(Object),!,
    rdf_assert(Subject, Predicate, Object, Graph).

%%
sw_assert_dataproperty(Subject, Predicate, Literal, Graph) :-
    rdf_is_literal(Literal),
    !,
    rdf_assert(Subject, Predicate, Literal, Graph).

sw_assert_dataproperty(Subject, Predicate, TypedValue, Graph) :-
    compound(TypedValue),
    TypedValue =.. [TypeName,Value],
    xsd_data_basetype(XSDType, TypeName),
    !,
    ( atom(Value) -> ValueAtom=Value ; term_to_atom(Value, ValueAtom) ),
    rdf_assert(Subject, Predicate, literal(type(XSDType, ValueAtom)), Graph).

sw_assert_dataproperty(Subject, Predicate, Atomic, Graph) :-
    atomic(Atomic),!,
	(   (   rdf_has(Predicate, rdfs:range, XSDType),
	        xsd_data_type(XSDType)
	    )
	;   xsd_guess_type(Atomic, XSDType)
	),
	term_to_atom(Atomic, ValueAtom),
    rdf_assert(Subject, Predicate, literal(type(XSDType, ValueAtom)), Graph).

%% sw_assert_type(+Resource, +Class) is semidet.
%
% Same as sw_assert_type/4 but only asserts universal (unscoped) facts
% into see default graph (see sw_default_graph/1).
%
sw_assert_type(Resource, Class) :-
    sw_default_graph(Graph),
    sw_assert_type(Resource, Class, Graph).

%% sw_assert_type(+Resource, +Class, +Graph) is semidet.
%
% Same as sw_assert_type/4 but only asserts universal (unscoped) facts.
%
sw_assert_type(Resource, Class, Graph) :-
    %sw_universal_scope(Scope),
    sw_assert_type(Resource, Class, Graph, _).

%% sw_assert_type(+Resource, +Class, +Graph, +Scope) is semidet.
%
% Assert a new triple into the database that uses the rdf:type predicate.
%
sw_assert_type(Resource, Class, Graph, _Scope) :-
    % todo: support class expressions
    % todo: support scoped classification
    atom(Class),
    atom(Resource),!,
    rdf_assert(Resource, rdf:type, Class, Graph).

		 /*******************************
		  *       ONTOLOGY URLs         *
		  *******************************/

%% sw_url(+URL, ?Resolved, ?OntologyGraph, ?OntologyVersion) is semidet.
%
sw_url(URL, Resolved, OntologyGraph, OntologyVersion) :-
	(	url_resolve(URL,Resolved)
	->	log_debug(db(url_resolved(URL,Resolved)))
	;	Resolved=URL 
	),
	sw_url_version(Resolved, OntologyVersion),
	sw_url_graph(Resolved, OntologyGraph).

%% sw_url_graph(+URL,?Name) is det.
%
% Each ontology file is stored in a separate graph named
% according to the ontology. The name is extracted as the basename
% of the URL without file type extension.
%

%% sw_url_version(+URL, ?Version) is det.
%
% Maps an ontology URL to a version atom.
% If URL is a local file, then the file modification time
% is used as version to cause relaoding on file modification.
% Else, it is attemptep to extract the version from the URI path.
% If this fails, the current date is used such that the file
% is reloaded on each day once.
%

%%
sw_register_prefix(Prefix, URI) :-
    rdf_register_prefix(Prefix, URI, [force(true)]),
    knowrob_register_namespace(Prefix, URI).

% register some common RDF namespaces
:- sw_register_prefix(dul,  'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#').
:- sw_register_prefix(soma, 'http://www.ease-crc.org/ont/SOMA.owl#').
:- sw_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').

%% sw_url_register_ns(+URL,+Options) is semidet.
%
% @param URL URL of a RDF file.
% @param Options List of options.
%
sw_url_register_ns(URL, Opts) :-
	% register namespace
	(	option(namespace(NS), Opts)
	->	(	atom_concat(URL, '#', Prefix),
			sw_register_prefix(NS, Prefix)
		)
	;	true
	),
	(	option(namespace(NS,Prefix), Opts)
	->	sw_register_prefix(NS, Prefix)
	;	true
	).

%%
is_annotation_triple(_, triple(_,P,_)) :-
	annotation_property(P),!.
is_annotation_triple(Terms, triple(_,P,_)) :-
	rdf_equal(rdf:'type',RDF_Type),
	rdf_equal(owl:'AnnotationProperty', AnnotationProperty),
	memberchk(triple(P, RDF_Type, string(AnnotationProperty)), Terms),!.

%%
convert_rdf_(IRI, rdf(S,P,O), triple(S1,P,O2)) :-
	convert_blank_node_(IRI,S,S1),
	convert_blank_node_(IRI,O,O1),
	convert_rdf_value_(O1,O2).

% avoid name clashes between blanks loaded from different ontologies
% by prepending the ontology prefix. 
convert_blank_node_(IRI,Blank,Converted) :-
	(	is_blank(Blank)
	->	atomic_list_concat([IRI,'#',Blank],'',Converted)
	;	Converted=Blank
	).

%%
is_blank(Blank) :-
	atom(Blank),
	atom_concat('_:',_,Blank).

% convert typed literals to expected format
convert_rdf_value_(literal(type(Type,V_atom)), O_typed) :-
	xsd_data_basetype(Type, TypeKey),
	(	TypeKey=integer -> atom_number(V_atom,V_typed)
	;	TypeKey=double  -> atom_number(V_atom,V_typed)
	;	V_typed=V_atom
	),
	O_typed=..[TypeKey,V_typed],
	!.

convert_rdf_value_(literal(O), string(O)) :- !.
convert_rdf_value_(        O,  string(O)) :- !.


     /*******************************
      *       GRAPH HIERARCHY       *
      *******************************/

%%
sw_current_graph(_, common) :- !.
sw_current_graph(_, user) :- !.
sw_current_graph(Reasoner, GraphName) :-
    current_reasoner_manager(ReasonerManager),
	sw_current_graph_cpp(ReasonerManager, Reasoner, GraphName).

%%
sw_set_current_graph(Reasoner, GraphName) :-
    current_reasoner_manager(ReasonerManager),
	sw_set_current_graph_cpp(ReasonerManager, Reasoner, GraphName).

%% sw_graph_includes(?Graph, ?IncludedGraph) is nondet.
%
sw_graph_includes(Graph, IncludedGraph) :-
    current_reasoner_manager(ReasonerManager),
    current_reasoner_module(Reasoner),
	sw_graph_get_imports_cpp(ReasonerManager, Reasoner, Graph, IncludedGraphs),
	member(IncludedGraph, IncludedGraphs).

%% sw_graph_include(+Graph,+IncludedGraph) is det.
%
% Adds the subgraph-of relation between two named graphs.
%
% @param Sub Name of the subgraph.
% @param Sup Name of the super-graph.
%
sw_graph_include(Graph, IncludedGraph) :-
    current_reasoner_manager(ReasonerManager),
    current_reasoner_module(Reasoner),
	sw_graph_add_direct_import_cpp(ReasonerManager, Reasoner, Graph, IncludedGraph).

%%
%
sw_unload_graph(Graph) :-
    current_reasoner_manager(ReasonerManager),
    current_reasoner_module(Reasoner),
	sw_unset_current_graph_cpp(ReasonerManager, Reasoner, Graph),
    rdf_unload_graph(Graph).

%% sw_default_graph(?Graph) is det.
%
% True if Graph is the current default graph.
%
sw_default_graph(Graph) :-
    current_reasoner_manager(ReasonerManager),
    current_reasoner_module(Reasoner),
	semweb:sw_default_graph_cpp(ReasonerManager, Reasoner, Graph).

%%
% Set the name of the graph where facts are asserted and retrieved
% if no other graph was specified.
%
sw_set_default_graph(Graph) :-
    current_reasoner_manager(ReasonerManager),
    current_reasoner_module(Reasoner),
	sw_set_default_graph_cpp(ReasonerManager, Reasoner, Graph).

     /*******************************
      *    LOADING RDF/XML DATA     *
      *******************************/

%%
%
load_rdf_xml(URL, ParentGraph) :-
    % make sure reasoner module defines rdf predicates
    current_reasoner_module(Reasoner),
    reasoner_rdf_init(Reasoner),
    % load rdf file
    load_rdf_xml1(URL, ParentGraph).

load_rdf_xml1(URL, ParentGraph) :-
    current_reasoner_manager(ReasonerManager),
    current_reasoner_module(Reasoner),
    sw_load_rdf_xml_cpp(ReasonerManager, Reasoner, URL, ParentGraph).

%load_rdf_xml1(URL, ParentGraph) :-
%	rdf_equal(owl:'imports', OWL_Imports),
%	rdf_equal(owl:'Ontology',OWL_Ontology),
%	rdf_equal(rdf:'type',RDF_Type),
%	% resolve URL, and read ontology graph name and version
%	sw_url(URL, Resolved, OntologyGraph, OntologyVersion),
%	% include ontology when parent graph is queried
%	sw_graph_include(ParentGraph, OntologyGraph),
%	% load RDF data
%	setup_call_cleanup(
%	    sw_url_stream(Resolved, QueryStage),
%	    rdf_load(QueryStage, [graph(OntologyGraph), silent(true)]),
%	    close(QueryStage)
%	),
%	% remember reasoner to graph association
%	current_reasoner_module(Reasoner),
%	sw_set_current_graph(Reasoner, OntologyGraph),
%	% lookup ontology URL
%	(	rdf(AssertedURL, RDF_Type, OWL_Ontology, OntologyGraph) -> true
%	;	log_error_and_fail(type_error(ontology,URL))
%	),
%	% load RDF data of imported ontologies
%	forall(
%		rdf(AssertedURL, OWL_Imports, ImportedURL, OntologyGraph),
%		load_rdf_xml1(ImportedURL, ParentGraph)
%	),
%	!,
%	log_debug(prolog(ontology_loaded(OntologyGraph,OntologyVersion))).

     /*******************************
     *          UNIT TESTS          *
     *******************************/

:- use_module(library('rdf_test')).
:- begin_rdf_tests('semweb', 'owl/test/pancake.owl').

:- sw_register_prefix(test, 'http://knowrob.org/kb/pancake.owl#').

test('sw_class_expr(+,-)') :-
    assert_true(sw_class_expr(test:'FlippingAPancake',_)),
    (   sw_class_expr(test:'FlippingAPancake',Expr)
    ->  assert_equals(Expr, class(test:'FlippingAPancake'))
    ;   true
    ).

test('sw_class_expr(+,+)') :-
    assert_true(sw_class_expr(test:'FlippingAPancake', class(test:'FlippingAPancake'))).

test('sw_subclass_of_expr(+,-)') :-
    findall(Expr, sw_subclass_of_expr(test:'FlippingAPancake', Expr), ExprList),
    assert_true(length(ExprList, 2)),
    assert_true(once(member(class(dul:'Task'),ExprList))),
    assert_true(once(member(exactly(dul:isTaskOf, 1, _),ExprList))).

test('sw_subclass_of_expr(+,+)') :-
    assert_true(sw_subclass_of_expr(test:'FlippingAPancake', class(dul:'Task'))),
    assert_true(sw_subclass_of_expr(test:'FlippingAPancake',
        exactly(dul:isTaskOf, 1, intersection_of( [class(soma:'Tool'), only(_,_)])))).

:- end_rdf_tests('semweb').
