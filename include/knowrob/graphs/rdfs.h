//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_RDFS_H
#define KNOWROB_RDFS_H

#include "string"

namespace knowrob::rdfs {
    const std::string IRI_subClassOf = "http://www.w3.org/2000/01/rdf-schema#subClassOf";
    const std::string IRI_subPropertyOf = "http://www.w3.org/2000/01/rdf-schema#subPropertyOf";

    const std::string IRI_comment = "http://www.w3.org/2000/01/rdf-schema#comment";
    const std::string IRI_seeAlso = "http://www.w3.org/2000/01/rdf-schema#seeAlso";
    const std::string IRI_label = "http://www.w3.org/2000/01/rdf-schema#label";

    bool isSubClassOfIRI(std::string_view iri) { return iri == IRI_subClassOf; }
    bool isSubPropertyOfIRI(std::string_view iri) { return iri == IRI_subPropertyOf; }
} // knowrob::rdf

#endif //KNOWROB_RDFS_H
