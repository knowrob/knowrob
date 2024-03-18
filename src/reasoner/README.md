\page reasoner Reasoner

KnowRob uses the Web Ontology Language (OWL) in combination with
rule based reasoning powered by SWI Prolog.
OWL ontologies can be parsed (owl_parse/1) which causes new facts to be
asserted in the RDF triple store of SWI Prolog.
SWI Prolog already provides querying with RDFS semantics
(rdf_has/3, rdfs_individual_of/2).

Ontop of the RDFS semantics KnowRob implements a minimal OWL reasoner in Prolog.
owl_individual_of/2 checks if a resource satisfies a class description
according to OWL-Description entailment rules,
and owl_has/3 checks if a relation can be deduced using OWL inference rules.
Some OWL2 features such as property chains are also supported,
but the reasoner is not OWL2 complete.

RDF triples may concretely be represented in the RDF triple store of SWI Prolog.
But KnowRob supports multiple sources for triples including
computational methods that ground relations in the data structures of the robot's
control program, or in its senses.
This concept is called *virtual knowledge base* in KnowRob.
Several virtual triple sources are pre-defined in KnowRob
including computable properties for
qualitative spatial reasoning (comp_spatial),
temporal reasoning (knowrob_common),
ROS tf data (knowrob_memory, knowrob_mongo),
and SWRL rules (swrl).

KnowRob is also capable to handle temporal information,
allowing robots to reason about past events.
Please note that OWL is generally considered inappropriate for
explicating time because it is limited to binary relations.
One approach to handle time is to *reify* relations to concepts
that also have time parameters.
The ontology used by KnowRob comes with a few reification concepts
that are handled within KnowRob.
However, KnowRob also supports explicit time in its QA interface. The time
specified is used as parameter for the triple store
(or to select appropiate reifications of relations).
In case of the default RDF triple store of SWI Prolog,
all triples are assumed to hold forever.
However, a dedicated *temporalized* triple store can be used
to represent triples that only hold limited time.
The time value specified in the query is used as parameter
for the triple store in that case.
knowrob_mem implements such a temporalized triple store.
The main interface to access temporalized triples is the
holds/2 predicate:

    holds(knowrob:volumeOfObject(Obj, 15.0), 25.0)

### Available Reasoning Components

The following reasoner are available in KnowRob:

- \subpage prolog
- \subpage mongolog
- \subpage owl
- \subpage esg
- \subpage srdl
- \subpage swrl
