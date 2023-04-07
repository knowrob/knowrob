//
// Created by daniel on 07.04.23.
//

#include "knowrob/semweb/rdfs.h"

namespace knowrob::semweb {
    bool isSubClassOfIRI(std::string_view iri)
    { return iri == IRI_subClassOf; }

    bool isSubPropertyOfIRI(std::string_view iri)
    { return iri == IRI_subPropertyOf; }

} // knowrob::semweb
