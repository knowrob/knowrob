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
     */
    class Vocabulary {
    public:
        Vocabulary();

        /**
         * Adds a new type assertion.
         * @param resource_iri a resource
         * @param type_iri the type of the resource
         */
        void addResourceType(std::string_view resource_iri, std::string_view type_iri);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined class
         */
        bool isDefinedClass(std::string_view iri);

        /**
         * @param iri a IRI
         * @return the class defined for the IRI or a null reference
         */
        ClassPtr getDefinedClass(std::string_view iri) const;

        /**
         * Define a new class if it has not been defined before.
         * @param iri a IRI
         * @return the class defined for the IRI
         */
        ClassPtr defineClass(std::string_view iri);

        /**
         * Adds subclass-of relation between two classes,
         * and defines them if they are not yet defined.
         * @param subClass a IRI
         * @param superClass a IRI
         */
        void addSubClassOf(std::string_view subClass, std::string_view superClass);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined property
         */
        bool isDefinedProperty(std::string_view iri);

        /**
         * @param iri a IRI
         * @return the property defined for the IRI or a null reference
         */
        PropertyPtr getDefinedProperty(std::string_view iri) const;

        /**
         * Define a new property if it has not been defined before.
         * @param iri a IRI
         * @return the property defined for the IRI
         */
        PropertyPtr defineProperty(std::string_view iri);

        /**
         * Adds subproperty-of relation between two properties,
         * and defines them if they are not yet defined.
         * @param subProperty a IRI
         * @param superProperty a IRI
         */
        void addSubPropertyOf(std::string_view subProperty, std::string_view superProperty);

        /**
         * @param iri a property IRI
         * @param flag a property flag
         */
        void setPropertyFlag(std::string_view iri, PropertyFlag flag);

        /**
         * @param iri a IRI
         * @return true if IRI is a defined annotation property
         */
        bool isAnnotationProperty(std::string_view iri);

    protected:
        std::map<std::string_view, ClassPtr> definedClasses_;
        std::map<std::string_view, PropertyPtr> definedProperties_;
    };

    using VocabularyPtr = std::shared_ptr<Vocabulary>;

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_VOCABULARY_H
