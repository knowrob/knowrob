/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/owl.h"

namespace knowrob {
    bool isClassIRI(std::string_view iri)
    { return iri == owl::Class->stringForm(); }

    bool isInverseOfIRI(std::string_view iri)
    { return iri == owl::inverseOf->stringForm(); }

    bool isObjectPropertyIRI(std::string_view iri)
    { return iri == owl::ObjectProperty->stringForm(); }

    bool isDatatypePropertyIRI(std::string_view iri)
    { return iri == owl::DatatypeProperty->stringForm(); }

    bool isAnnotationPropertyIRI(std::string_view iri)
    { return iri == owl::AnnotationProperty->stringForm(); }

    bool isTransitivePropertyIRI(std::string_view iri)
    { return iri == owl::TransitiveProperty->stringForm(); }

    bool isSymmetricPropertyIRI(std::string_view iri)
    { return iri == owl::SymmetricProperty->stringForm(); }

    bool isReflexivePropertyIRI(std::string_view iri)
    { return iri == owl::ReflexiveProperty->stringForm(); }
} // knowrob::semweb
