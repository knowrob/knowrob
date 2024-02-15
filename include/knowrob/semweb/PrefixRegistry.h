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
        void registerPrefix(std::string_view prefix, std::string_view uri);

		/**
		 * Maps URI to alias.
		 * @param uri a URI
		 * @return an alias, or nullopt if no alias is known.
		 */
        OptionalStringRef uriToAlias(const std::string &uri) const;

		/**
		 * Maps alias to URI.
		 * @param alias a URI alias
		 * @return the corresponding URI, or nullopt if alias is unknown.
		 */
        OptionalStringRef aliasToUri(const std::string &alias) const;

        /**
         * Construct a full IRI from alias and entity name.
         * @param alias namespace alias
         * @param entityName the name of an entity in the namespace identified by alias
         * @return a full IRI if namespace alias is known
         */
        std::optional<std::string> createIRI(const std::string &alias, const std::string &entityName) const;

        /**
         * @param prefix a alias prefix
         * @return all aliases with matching prefix
         */
        std::vector<std::string_view> getAliasesWithPrefix(const std::string &prefix) const;

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
        std::map<std::string, std::string, std::less<>> aliasToURI_;
    };
}


#endif //KNOWROB_SEMWEB_PREFIX_REGISTRY_H
