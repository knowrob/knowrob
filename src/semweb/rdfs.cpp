//
// Created by daniel on 07.04.23.
//

#include "knowrob/semweb/rdfs.h"

namespace knowrob::semweb {
    bool isSubClassOfIRI(std::string_view iri)
    { return iri == rdfs::subClassOf; }

    bool isSubPropertyOfIRI(std::string_view iri)
    { return iri == rdfs::subPropertyOf; }

} // knowrob::semweb
