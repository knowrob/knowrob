//
// Created by daniel on 07.04.23.
//

#include "knowrob/semweb/owl.h"

namespace knowrob::semweb {
    bool isClassIRI(std::string_view iri)
    { return iri == IRI_Class; }

    bool isInverseOfIRI(std::string_view iri)
    { return iri == IRI_inverseOf; }

    bool isObjectPropertyIRI(std::string_view iri)
    { return iri == IRI_ObjectProperty; }

    bool isDatatypePropertyIRI(std::string_view iri)
    { return iri == IRI_DatatypeProperty; }

    bool isAnnotationPropertyIRI(std::string_view iri)
    { return iri == IRI_AnnotationProperty; }

    bool isTransitivePropertyIRI(std::string_view iri)
    { return iri == IRI_TransitiveProperty; }

    bool isSymmetricPropertyIRI(std::string_view iri)
    { return iri == IRI_SymmetricProperty; }

    bool isReflexivePropertyIRI(std::string_view iri)
    { return iri == IRI_ReflexiveProperty; }
} // knowrob::semweb
