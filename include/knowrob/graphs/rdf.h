//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_RDF_H
#define KNOWROB_RDF_H

#include "string"

namespace knowrob::rdf {
    const std::string IRI_type = "http://www.w3.org/1999/02/22-rdf-syntax-ns#type";

    bool isTypeIRI(std::string_view iri) {
        return iri == IRI_type;
    }
} // knowrob::rdf

#endif //KNOWROB_RDF_H
