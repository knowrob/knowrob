:- module(model_RDFS,
    [ is_resource(r),
      is_property(r),
      is_literal(r),
      is_datatype(r),
      has_type(r,r),
      has_range(r,r),
      has_domain(r,r),
      has_label(r,?),
      has_comment(r,?),
      is_rdf_list(r,t)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db'),
    [ rdf_register_ns/3,
      rdf_current_ns/2,
      rdf_split_url/3
    ]).
:- rdf_register_ns(rdfs,
    'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
% TODO move somewhere else
:- rdf_register_ns(io,
    'http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl#', [keep(true)]).

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

%% has_type(+Resource,?Type) is semidet.
%
% rdf:type is an instance of rdf:Property that is used to
% state that a resource is an instance of a class.
%
has_type(Resource,Type) ?>
  triple(Resource, rdf:type, Type).

has_type(Resource,Type) +>
  %triple(Resource, rdf:type, Label).
  instance_of(Resource,Type).

%%
%
%
has_range(Property,Range) ?+>
  triple(Property, rdfs:range, Range).

%%
%
%
has_domain(Property,Range) ?+>
  triple(Property, rdfs:domain, Range).

%% has_label(+Resource,?Comment) is semidet.
%
% rdfs:label is an instance of rdf:Property that may be used
% to provide a human-readable version of a resource's name.
%
has_label(Resource,Label) ?+>
  triple(Resource, rdfs:label, Label).

%% has_comment(+Resource,?Comment) is semidet.
%
% rdfs:comment is an instance of rdf:Property that may be used
% to provide a human-readable description of a resource.
%
has_comment(Resource,Comment) ?+>
  triple(Resource, rdfs:comment, Comment).

%%
%
%
is_rdf_list('http://www.w3.org/1999/02/22-rdf-syntax-ns#nil',[]) ?+> { ! }.

is_rdf_list(RDFList,[X|Xs]) ?>
  triple(RDFList, rdf:first, X),
  triple(RDFList, rdf:rest, Ys),
  is_rdf_list(Ys,Xs).

is_rdf_list(RDFList,[X|Xs]) +>
  is_rdf_list(Ys,Xs),
  % TODO: might be overkill to have rdf:type for each sublist?
  has_type(RDFList, rdf:'List'),
  triple(RDFList, rdf:first, X),
  triple(RDFList, rdf:rest, Ys).
