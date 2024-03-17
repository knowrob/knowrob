/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/rdf.h"

namespace knowrob {
    bool isTypeIRI(std::string_view iri)
    { return iri == rdf::type->stringForm(); }

    bool isPropertyIRI(std::string_view iri)
    { return iri == rdf::Property->stringForm(); }

} // knowrob::semweb
