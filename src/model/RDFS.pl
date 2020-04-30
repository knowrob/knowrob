:- module(model_RDFS,
    [ is_resource(r),
      is_property(r),
      is_literal(r),
      is_datatype(r),
      is_subclass_of(r,r),
      is_subproperty_of(r,r),
      has_type(r,r),
      has_property_range(r,r),
      has_property_domain(r,r),
      has_label(r,?),
      has_comment(r,?),
      rdfs_list(r,t)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db'),
        [ rdf_register_ns/3 ]).

:- rdf_register_ns(rdfs,
        'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).

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
  has_type(Entity, rdfs:'Property').

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

%% is_subclass_of(+SubClass,?Class) is semidet.
%
% The property rdfs:subClassOf is an instance of rdf:Property
% that is used to state that all the instances of one class
% are instances of another.
%
is_subclass_of(SubClass,Class) ?+>
  holds(SubClass, rdfs:subClassOf, Class).

%% is_subproperty_of(+Resource,?Property) is semidet.
%
% The property rdfs:subPropertyOf is an instance of rdf:Property
% that is used to state that all resources related by one property
% are also related by another.
%
is_subproperty_of(SubProperty,Property) ?+>
  holds(SubProperty, rdfs:subPropertyOf, Property).

%% has_type(+Resource,?Type) is semidet.
%
% rdf:type is an instance of rdf:Property that is used to
% state that a resource is an instance of a class.
%
has_type(Resource,Type) ?+>
  holds(Resource, rdf:type, Label).

has_property_range(P,Range) ?+>
  tripledb_ask(P,rdfs:range,Range).

has_property_domain(P,Range) ?+>
  tripledb_ask(P,rdfs:domain,Range).

%% has_label(+Resource,?Comment) is semidet.
%
% rdfs:label is an instance of rdf:Property that may be used
% to provide a human-readable version of a resource's name.
%
has_label(Resource,Label) ?+>
  holds(Resource, rdfs:label, Label).

%% has_comment(+Resource,?Comment) is semidet.
%
% rdfs:comment is an instance of rdf:Property that may be used
% to provide a human-readable description of a resource.
%
has_comment(Resource,Comment) ?+>
  holds(Resource, rdfs:comment, Comment).

%%
%
%
rdfs_list(rdf:nil,[]) +?> { ! }.

rdfs_list(RDFList,[X|Xs]) ?>
  holds(RDFList, rdf:first, X),
  holds(RDFList, rdf:rdf:rest, Ys),
  rdfs_list(Ys,Xs).

rdfs_list(RDFList,[X|Xs]) +>
  rdfs_list(Ys,Xs),
  % TODO: might be overkill to have rdf:type for each sublist?
  has_type(RDFList, rdf:'List'),
  holds(RDFList,    rdf:first, X),
  holds(RDFList,    rdf:rest, Ys).
