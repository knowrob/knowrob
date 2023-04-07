//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_SEMWEB_RDFS_H
#define KNOWROB_SEMWEB_RDFS_H

#include "string"

namespace knowrob::semweb {
    const std::string IRI_subClassOf        = "http://www.w3.org/2000/01/rdf-schema#subClassOf";
    const std::string IRI_subPropertyOf     = "http://www.w3.org/2000/01/rdf-schema#subPropertyOf";

    const std::string IRI_comment           = "http://www.w3.org/2000/01/rdf-schema#comment";
    const std::string IRI_seeAlso           = "http://www.w3.org/2000/01/rdf-schema#seeAlso";
    const std::string IRI_label             = "http://www.w3.org/2000/01/rdf-schema#label";

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
