//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_RDF_RESOURCE_H
#define KNOWROB_RDF_RESOURCE_H

#include <string>

namespace knowrob::semweb {
    /**
     * A RDF resource.
     */
    class Resource {
    public:
        explicit Resource(std::string_view iri) : iri_(iri) {}

        /**
         * @return the IRI string of this resource
         */
        const auto& iri() const { return iri_; }

    protected:
        std::string iri_;
    };

} // knowrob::semweb

#endif //KNOWROB_RDF_RESOURCE_H
