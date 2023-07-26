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
		/**
		 * @return the singleton instance of the registry.
		 */
        static PrefixRegistry& get();

		/**
		 * Register a namespace prefix.
		 * This will overwrite any existing prefix registered for given URI.
		 * @param prefix a short name for the URI.
		 * @param uri a full URI.
		 */
        void registerPrefix(const std::string &prefix, const std::string &uri);

		/**
		 * Maps URI to alias.
		 * @param uri a URI
		 * @return an alias, or nullopt if no alias is known.
		 */
        OptionalStringRef uriToAlias(const std::string &uri);

		/**
		 * Maps alias to URI.
		 * @param alias a URI alias
		 * @return the corresponding URI, or nullopt if alias is unknown.
		 */
        OptionalStringRef aliasToUri(const std::string &alias);

		/**
		 * An iterator of the registry map.
		 */
        using Iterator = std::map<std::string, std::string>::iterator;

		/**
		 * @return begin iterator over the registry map.
		 */
        Iterator begin() { return uriToAlias_.begin(); }

		/**
		 * @return end iterator over the registry map.
		 */
        Iterator end() { return uriToAlias_.end(); }
    private:
        PrefixRegistry();

        std::map<std::string, std::string> uriToAlias_;
        std::map<std::string, std::string> aliasToURI_;
    };
}


#endif //KNOWROB_SEMWEB_PREFIX_REGISTRY_H
