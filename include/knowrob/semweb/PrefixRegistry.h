//
// Created by daniel on 11.02.23.
//

#ifndef KNOWROB_SEMWEB_PREFIX_REGISTRY_H
#define KNOWROB_SEMWEB_PREFIX_REGISTRY_H

#include <string>
#include <map>
#include <functional>
#include <optional>

namespace knowrob::semweb {
    using OptionalStringRef = std::optional<std::reference_wrapper<const std::string>>;

    /**
     * Stores short names of IRI prefixes.
     */
    class PrefixRegistry {
    public:
        static PrefixRegistry& get();

        void registerPrefix(const std::string &prefix, const std::string &uri);

        OptionalStringRef uriToAlias(const std::string &uri);

        OptionalStringRef aliasToUri(const std::string &alias);

        using Iterator = std::map<std::string, std::string>::iterator;

        Iterator begin() { return uriToAlias_.begin(); }

        Iterator end() { return uriToAlias_.end(); }
    private:
        PrefixRegistry();

        std::map<std::string, std::string> uriToAlias_;
        std::map<std::string, std::string> aliasToURI_;
    };
}


#endif //KNOWROB_SEMWEB_PREFIX_REGISTRY_H
