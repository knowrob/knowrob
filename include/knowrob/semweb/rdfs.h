/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_RDFS_H
#define KNOWROB_SEMWEB_RDFS_H

#include "string"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob {
    namespace rdfs {
        constexpr std::string_view prefix  = "http://www.w3.org/2000/01/rdf-schema#";
        const IRIAtomPtr subClassOf        = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#subClassOf");
        const IRIAtomPtr subPropertyOf     = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#subPropertyOf");
        const IRIAtomPtr comment           = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#comment");
        const IRIAtomPtr seeAlso           = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#seeAlso");
        const IRIAtomPtr label             = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#label");
        const IRIAtomPtr range             = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#range");
        const IRIAtomPtr domain            = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#domain");
        const IRIAtomPtr Class             = IRIAtom::Tabled("http://www.w3.org/2000/01/rdf-schema#Class");
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
