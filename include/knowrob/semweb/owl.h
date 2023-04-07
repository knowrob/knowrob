//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_SEMWEB_OWL_H
#define KNOWROB_SEMWEB_OWL_H

#include "string"

namespace knowrob::semweb {
    const std::string IRI_imports               = "http://www.w3.org/2002/07/owl#imports";
    const std::string IRI_versionInfo           = "http://www.w3.org/2002/07/owl#versionInfo";
    const std::string IRI_inverseOf             = "http://www.w3.org/2002/07/owl#inverseOf";
    const std::string IRI_Class                 = "http://www.w3.org/2002/07/owl#Class";
    const std::string IRI_ObjectProperty        = "http://www.w3.org/2002/07/owl#ObjectProperty";
    const std::string IRI_DatatypeProperty      = "http://www.w3.org/2002/07/owl#DatatypeProperty";
    const std::string IRI_AnnotationProperty    = "http://www.w3.org/2002/07/owl#AnnotationProperty";
    const std::string IRI_TransitiveProperty    = "http://www.w3.org/2002/07/owl#TransitiveProperty";
    const std::string IRI_SymmetricProperty     = "http://www.w3.org/2002/07/owl#SymmetricProperty";
    const std::string IRI_ReflexiveProperty     = "http://www.w3.org/2002/07/owl#ReflexiveProperty";

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
