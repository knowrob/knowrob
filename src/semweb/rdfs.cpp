/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/rdfs.h"

namespace knowrob {
    bool isSubClassOfIRI(std::string_view iri)
    { return iri == rdfs::subClassOf->stringForm(); }

    bool isSubPropertyOfIRI(std::string_view iri)
    { return iri == rdfs::subPropertyOf->stringForm(); }

} // knowrob::semweb
