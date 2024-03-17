/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_RDF_H
#define KNOWROB_SEMWEB_RDF_H

#include "string"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob {
    namespace rdf {
        constexpr std::string_view prefix    = "http://www.w3.org/1999/02/22-rdf-syntax-ns#";
        const IRIAtomPtr type      = IRIAtom::Tabled("http://www.w3.org/1999/02/22-rdf-syntax-ns#type");
        const IRIAtomPtr Property  = IRIAtom::Tabled("http://www.w3.org/1999/02/22-rdf-syntax-ns#Property");
    }

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
