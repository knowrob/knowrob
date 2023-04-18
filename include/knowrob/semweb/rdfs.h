//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_SEMWEB_RDFS_H
#define KNOWROB_SEMWEB_RDFS_H

#include "string"

namespace knowrob::semweb {
    namespace rdfs {
        constexpr std::string_view prefix            = "http://www.w3.org/2000/01/rdf-schema#";
        constexpr std::string_view subClassOf        = "http://www.w3.org/2000/01/rdf-schema#subClassOf";
        constexpr std::string_view subPropertyOf     = "http://www.w3.org/2000/01/rdf-schema#subPropertyOf";
        constexpr std::string_view comment           = "http://www.w3.org/2000/01/rdf-schema#comment";
        constexpr std::string_view seeAlso           = "http://www.w3.org/2000/01/rdf-schema#seeAlso";
        constexpr std::string_view label             = "http://www.w3.org/2000/01/rdf-schema#label";
    }

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=rdfs:'subClassOf'
     */
    bool isSubClassOfIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=rdfs:'subPropertyOf'
     */
    bool isSubPropertyOfIRI(std::string_view iri);
} // knowrob::semweb

#endif //KNOWROB_SEMWEB_RDFS_H
