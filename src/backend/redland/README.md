\page redland_backend Redland Backend

A backend implementation for the KnowRob knowledge base using the Redland RDF library.
Note that only storage types that support context nodes can be used, as this is required
in order to store the "origin" of triples.
Additional contextual parameters (e.g. time, confidence, etc.) are not supported by this backend,
and thus need to be handled through reification (which is done automatically by KnowRob).
