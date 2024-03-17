/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_OWL_H
#define KNOWROB_SEMWEB_OWL_H

#include "string"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob {
    namespace owl {
        constexpr std::string_view prefix    = "http://www.w3.org/2002/07/owl#";
        const IRIAtomPtr imports             = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#imports");
        const IRIAtomPtr versionInfo         = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#versionInfo");
        const IRIAtomPtr inverseOf           = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#inverseOf");
        const IRIAtomPtr Class               = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#Class");
        const IRIAtomPtr Thing               = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#Thing");
        const IRIAtomPtr Restriction         = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#Restriction");
        const IRIAtomPtr ObjectProperty      = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#ObjectProperty");
        const IRIAtomPtr DatatypeProperty    = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#DatatypeProperty");
        const IRIAtomPtr AnnotationProperty  = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#AnnotationProperty");
        const IRIAtomPtr TransitiveProperty  = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#TransitiveProperty");
        const IRIAtomPtr SymmetricProperty   = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#SymmetricProperty");
        const IRIAtomPtr ReflexiveProperty   = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#ReflexiveProperty");
        const IRIAtomPtr NamedIndividual     = IRIAtom::Tabled("http://www.w3.org/2002/07/owl#NamedIndividual");
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
