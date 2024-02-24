/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_PROPERTY_H
#define KNOWROB_SEMWEB_PROPERTY_H

#include <memory>
#include <list>
#include <functional>
#include "Resource.h"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob::semweb {
    /**
     * Defines property type and semantics.
     */
    enum PropertyFlag {
        DATATYPE_PROPERTY       = 1 << 0,
        ANNOTATION_PROPERTY     = 1 << 1,
        OBJECT_PROPERTY         = 1 << 2,
        TRANSITIVE_PROPERTY     = 1 << 3,
        REFLEXIVE_PROPERTY      = 1 << 4,
        SYMMETRIC_PROPERTY      = 1 << 5
    };

    // forward declaration
    class Property;

    // called for each parent in the property hierarchy
    using PropertyVisitor = std::function<void(Property&)>;

    /**
     * A property used in knowledge graphs.
     */
    class Property : public Resource {
    public:
        explicit Property(std::string_view iri);

        explicit Property(const IRIAtomPtr &iri);

        /**
         * @param directParent a direct super property.
         */
        void addDirectParent(const std::shared_ptr<Property> &directParent);

        /**
         * @return all direct super properties of this property.
         */
        const auto& directParents() const { return directParents_; }

        /**
         * Define the inverse property of this property.
         * @param inverse a property.
         */
        void setInverse(const std::shared_ptr<Property> &inverse);

        /**
         * @return the inverse of this property or a null pointer reference.
         */
        const auto& inverse() const { return inverse_; }

        /**
         * @param flag a property flag.
         * @return true if this property has the flag.
         */
        bool hasFlag(PropertyFlag flag) const;

        /**
         * Define a flag of this property.
         * @param flag a property flag.
         */
        void setFlag(PropertyFlag flag);

        void forallParents(const PropertyVisitor &visitor, bool includeSelf=true, bool skipDuplicates=true);

    protected:
        std::shared_ptr<Property> inverse_;
        std::list<std::shared_ptr<Property>> directParents_;
        int flags_;
    };

    using PropertyPtr = std::shared_ptr<Property>;

} // knowrob

#endif //KNOWROB_SEMWEB_PROPERTY_H
