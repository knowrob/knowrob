//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_SEMWEB_RDF_H
#define KNOWROB_SEMWEB_RDF_H

#include "string"

namespace knowrob::semweb {
    const std::string IRI_type      = "http://www.w3.org/1999/02/22-rdf-syntax-ns#type";
    const std::string IRI_Property  = "http://www.w3.org/1999/02/22-rdf-syntax-ns#Property";

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=rdf:'Type'
     */
    bool isTypeIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=rdf:'Property'
     */
    bool isPropertyIRI(std::string_view iri);

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_RDF_H
