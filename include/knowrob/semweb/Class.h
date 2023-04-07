//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_SEMWEB_CLASS_H
#define KNOWROB_SEMWEB_CLASS_H

#include <memory>
#include <list>
#include <functional>
#include "Resource.h"

namespace knowrob::semweb {

    // forward declaration
    class Class;

    // called for each parent in the property hierarchy
    using ClassVisitor = std::function<void(Class&)>;

    /**
     * A RDF class.
     */
    class Class : public Resource {
    public:
        explicit Class(std::string_view iri);

        /**
         * @param directParent a direct super class.
         */
        void addDirectParent(const std::shared_ptr<Class> &directParent);

        /**
         * @return all direct super classes of this class.
         */
        const auto& directParents() const { return directParents_; }

        void forallParents(const ClassVisitor &visitor, bool includeSelf=true, bool skipDuplicates=true);

    protected:
        std::list<std::shared_ptr<Class>> directParents_;
    };

    using ClassPtr = std::shared_ptr<Class>;

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_CLASS_H
