//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_SEMWEB_OWL_H
#define KNOWROB_SEMWEB_OWL_H

#include "string"

namespace knowrob::semweb {
    namespace owl {
        constexpr std::string_view prefix                = "http://www.w3.org/2002/07/owl#";
        constexpr std::string_view imports               = "http://www.w3.org/2002/07/owl#imports";
        constexpr std::string_view versionInfo           = "http://www.w3.org/2002/07/owl#versionInfo";
        constexpr std::string_view inverseOf             = "http://www.w3.org/2002/07/owl#inverseOf";
        constexpr std::string_view Class                 = "http://www.w3.org/2002/07/owl#Class";
        constexpr std::string_view ObjectProperty        = "http://www.w3.org/2002/07/owl#ObjectProperty";
        constexpr std::string_view DatatypeProperty      = "http://www.w3.org/2002/07/owl#DatatypeProperty";
        constexpr std::string_view AnnotationProperty    = "http://www.w3.org/2002/07/owl#AnnotationProperty";
        constexpr std::string_view TransitiveProperty    = "http://www.w3.org/2002/07/owl#TransitiveProperty";
        constexpr std::string_view SymmetricProperty     = "http://www.w3.org/2002/07/owl#SymmetricProperty";
        constexpr std::string_view ReflexiveProperty     = "http://www.w3.org/2002/07/owl#ReflexiveProperty";
    }

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'Class'
     */
    bool isClassIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'inverseOf'
     */
    bool isInverseOfIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'ObjectProperty'
     */
    bool isObjectPropertyIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'AnnotationProperty'
     */
    bool isDatatypePropertyIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'Class'
     */
    bool isAnnotationPropertyIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'TransitiveProperty'
     */
    bool isTransitivePropertyIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'SymmetricProperty'
     */
    bool isSymmetricPropertyIRI(std::string_view iri);

    /**
     * @param iri the IRI of a RDF resource
     * @return true if iri=owl:'ReflexiveProperty'
     */
    bool isReflexivePropertyIRI(std::string_view iri);
} // knowrob::semweb

#endif //KNOWROB_SEMWEB_OWL_H
