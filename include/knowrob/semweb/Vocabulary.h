//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_SEMWEB_VOCABULARY_H
#define KNOWROB_SEMWEB_VOCABULARY_H

#include <memory>
#include <string>
#include <string_view>
#include <map>
#include "Property.h"
#include "Class.h"

namespace knowrob::semweb {
    /**
     * The vocabulary of a knowledge graph.
     * It contains the definitions of classes and properties.
     */
    class Vocabulary {
    public:
        Vocabulary();

        /**
         * Adds a new type assertion.
         * @param resource_iri a resource
         * @param type_iri the type of the resource
         */
        void addResourceType(const std::string_view &resource_iri, const std::string_view &type_iri);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined class
         */
        bool isDefinedClass(const std::string_view &iri);

        /**
         * @param iri a IRI
         * @return the class defined for the IRI or a null reference
         */
        ClassPtr getDefinedClass(const std::string_view &iri) const;

        /**
         * @param prefix a IRI prefix
         * @return all known classes with matching prefix
         */
        std::vector<ClassPtr> getDefinedClassesWithPrefix(const std::string_view &prefix) const;

		/**
		 * @param prefix a IRI prefix
		 * @return all known class names with matching prefix
		 */
        std::vector<std::string_view> getDefinedClassNamesWithPrefix(const std::string_view &prefix) const;

        /**
         * Define a new class if it has not been defined before.
         * @param iri a IRI
         * @return the class defined for the IRI
         */
        ClassPtr defineClass(const std::string_view &iri);

        /**
         * Adds subclass-of relation between two classes,
         * and defines them if they are not yet defined.
         * @param subClass a IRI
         * @param superClass a IRI
         */
        void addSubClassOf(const std::string_view &subClass, const std::string_view &superClass);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined property
         */
        bool isDefinedProperty(const std::string_view &iri);

        /**
         * @param iri a IRI
         * @return the property defined for the IRI or a null reference
         */
        PropertyPtr getDefinedProperty(const std::string_view &iri) const;

        /**
         * @param prefix a IRI prefix
         * @return all known properties with matching prefix
         */
        std::vector<PropertyPtr> getDefinedPropertiesWithPrefix(const std::string_view &prefix) const;

		/**
		 * @param prefix a IRI prefix
		 * @return all known property names with matching prefix
		 */
        std::vector<std::string_view> getDefinedPropertyNamesWithPrefix(const std::string_view &prefix) const;

        /**
         * Define a new property if it has not been defined before.
         * @param iri a IRI
         * @return the property defined for the IRI
         */
        PropertyPtr defineProperty(const std::string_view &iri);

        /**
         * Adds subproperty-of relation between two properties,
         * and defines them if they are not yet defined.
         * @param subProperty a IRI
         * @param superProperty a IRI
         */
        void addSubPropertyOf(const std::string_view &subProperty, const std::string_view &superProperty);

        /**
         * Define inverseOf relation between properties.
         * @param a a property IRI
         * @param b a property IRI
         */
        void setInverseOf(const std::string_view &a, const std::string_view &b);

        /**
         * @param iri a property IRI
         * @param flag a property flag
         */
        void setPropertyFlag(const std::string_view &iri, PropertyFlag flag);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined annotation property
         */
        bool isAnnotationProperty(const std::string_view &iri);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined taxonomic property
         */
        static bool isTaxonomicProperty(const std::string_view &iri);

		/**
		 * Set the frequency of a resource reflecting how often it appears
		 * in the knowledge corpus.
		 * @param iri a IRI
		 * @param frequency a frequency
		 */
        void setFrequency(const std::string_view &iri, uint32_t frequency);

        /**
         * Increase the frequency of a resource by one reflecting that it appears
         * one more time in the knowledge corpus.
         * @param iri a IRI
         */
        void increaseFrequency(const std::string_view &iri);

        /**
         * Returns the frequency of a resource reflecting how often it appears
         * in the knowledge corpus.
         * @param iri a IRI
         * @return the frequency
         */
        uint32_t frequency(const std::string_view &iri) const;

    protected:
        std::map<std::string_view, ClassPtr, std::less<>> definedClasses_;
        std::map<std::string_view, PropertyPtr, std::less<>> definedProperties_;
        std::map<std::string_view, uint32_t> frequency_;
    };

    using VocabularyPtr = std::shared_ptr<Vocabulary>;

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_VOCABULARY_H
