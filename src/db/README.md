knowrob_db
=======

KnowRob distinguishes between two different types of databases:
The triple DB which is used to store factual knowledge,
and DBs which are used to store quantitative information.
Interfaces are defined for the both distinct cases such
that it is possible to configure the triple DB used by KnowRob,
and to dynamically link the knowledge base to new sources
of quantitative information.

### triple DB

The triple store holds the RDF data of the knowledge base.
However, KnowRob requires a storage that allows to associate
additional information to triples stored in the DB.
In particular, triples in KnowRob are (temporally) scoped,
meaning that they are only true within a limited scope.
KnowRob may further associate a unit to data values,
which must also be associated to the corresponding data triple.

Another requirement is that the triple DB supports RDFS semantics
directly -- so to have very fast retrieval of subclass-of and
subproperty-of relationships.

Finally, triples are stored in named graphs.
This allows to separate triple data from distinct contexts.
The graphs in the triple store form a subgraph-of hierarchy:
triples of the supergraphs are included in subgraphs.
So to allow common super graphs shared in different contexts.

### OBDA

Quantitative information is incorporated into the knowledge base
through ontology-based data access (OBDA).
KnowRob defines a common interface for OBDA sources where
each source declares a binary predicate that maps to a binary
relation symbol.
More specifically, each OBDA predicate maps to some sub-property
of the relation hasDataValue (a datatype property).
This mapping is used by KnowRob to incorporate OBDA sources
into question answering where questions are structured according to
the formal model used by KnowRob.
