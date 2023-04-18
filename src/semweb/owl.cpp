//
// Created by daniel on 07.04.23.
//

#include "knowrob/semweb/owl.h"

namespace knowrob::semweb {
    bool isClassIRI(std::string_view iri)
    { return iri == owl::Class; }

    bool isInverseOfIRI(std::string_view iri)
    { return iri == owl::inverseOf; }

    bool isObjectPropertyIRI(std::string_view iri)
    { return iri == owl::ObjectProperty; }

    bool isDatatypePropertyIRI(std::string_view iri)
    { return iri == owl::DatatypeProperty; }

    bool isAnnotationPropertyIRI(std::string_view iri)
    { return iri == owl::AnnotationProperty; }

    bool isTransitivePropertyIRI(std::string_view iri)
    { return iri == owl::TransitiveProperty; }

    bool isSymmetricPropertyIRI(std::string_view iri)
    { return iri == owl::SymmetricProperty; }

    bool isReflexivePropertyIRI(std::string_view iri)
    { return iri == owl::ReflexiveProperty; }
} // knowrob::semweb
