swrlprolog
===

Implementation of a Semantic Web Rule Language (SWRL) reasoner in a set of Prolog rules.
SWRL rules are either described in RDF concrete syntax, or using a custom file
format.

The SWRL reasoner either projects inferred triples into the RDF store
of the knowledge base, or the inference rules are used to extend
KnowRob's virtual knowledge bases, providing additional triples for the layers above.
