//
// Created by daniel on 07.04.23.
//

#include "knowrob/semweb/rdf.h"

namespace knowrob::semweb {
    bool isTypeIRI(std::string_view iri)
    { return iri == rdf::type; }

    bool isPropertyIRI(std::string_view iri)
    { return iri == rdf::Property; }

} // knowrob::semweb
