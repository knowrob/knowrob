/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PREFIX_REGISTRY_H
#define KNOWROB_PREFIX_REGISTRY_H

#include <string>
#include <map>
#include <functional>
#include <optional>

namespace knowrob {
	using OptionalStringRef = std::optional<std::reference_wrapper<const std::string>>;

	/**
	 * Stores short names of IRI prefixes.
	 */
	class PrefixRegistry {
	public:
		/**
		 * @return the singleton instance of the registry.
		 */
		static PrefixRegistry &get();

		/**
		 * Register a namespace prefix.
		 * This will overwrite any existing prefix registered for given URI.
		 * @param prefix a short name for the URI.
		 * @param uri a full URI.
		 */
		static void registerPrefix(std::string_view prefix, std::string_view uri);

		/**
		 * Maps URI to alias.
		 * @param uri a URI
		 * @return an alias, or nullopt if no alias is known.
		 */
		static OptionalStringRef uriToAlias(std::string_view uri);

		/**
		 * Maps alias to URI.
		 * @param alias a URI alias
		 * @return the corresponding URI, or nullopt if alias is unknown.
		 */
		static OptionalStringRef aliasToUri(std::string_view alias);

		/**
		 * Construct a full IRI from alias and entity name.
		 * @param alias namespace alias
		 * @param entityName the name of an entity in the namespace identified by alias
		 * @return a full IRI if namespace alias is known
		 */
		static std::optional<std::string> createIRI(std::string_view alias, std::string_view entityName);

		/**
		 * @param prefix a alias prefix
		 * @return all aliases with matching prefix
		 */
		static std::vector<std::string_view> getAliasesWithPrefix(std::string_view prefix);

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
		std::map<std::string, std::string, std::less<>> uriToAlias_;
		std::map<std::string, std::string, std::less<>> aliasToURI_;

		PrefixRegistry();

		void registerPrefix_(std::string_view prefix, std::string_view uri);
	};
}


#endif //KNOWROB_PREFIX_REGISTRY_H
