//
// Created by daniel on 11.02.23.
//

#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/semweb/PrefixProbe.h"
#include "knowrob/Logger.h"

using namespace knowrob::semweb;

PrefixRegistry::PrefixRegistry() {
	registerPrefix("owl", "http://www.w3.org/2002/07/owl");
	registerPrefix("rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns");
	registerPrefix("rdfs", "http://www.w3.org/2000/01/rdf-schema");
	registerPrefix("xsd", "http://www.w3.org/2001/XMLSchema");
	registerPrefix("dul", "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl");
}

PrefixRegistry &PrefixRegistry::get() {
	static PrefixRegistry singleton;
	return singleton;
}

void PrefixRegistry::registerPrefix(std::string_view prefix, std::string_view uri) {
	auto s_prefix = std::string(prefix.data());
	auto s_uri = std::string(uri.data());
	if (s_uri[s_uri.size() - 1] == '#') {
		s_uri.pop_back();
	}
	uriToAlias_[s_uri] = s_prefix;
	aliasToURI_[s_prefix] = s_uri;
}

OptionalStringRef PrefixRegistry::uriToAlias(const std::string &uri) const {
	if (uri[uri.size() - 1] == '#') {
		auto x = uri;
		x.pop_back();
		auto it = uriToAlias_.find(x);
		return it == uriToAlias_.end() ? std::nullopt : OptionalStringRef(it->second);
	} else {
		auto it = uriToAlias_.find(uri);
		return it == uriToAlias_.end() ? std::nullopt : OptionalStringRef(it->second);
	}
}

OptionalStringRef PrefixRegistry::aliasToUri(const std::string &alias) const {
	auto it = aliasToURI_.find(alias);
	return it == aliasToURI_.end() ? std::nullopt : OptionalStringRef(it->second);
}

std::optional<std::string> PrefixRegistry::createIRI(const std::string &alias, const std::string &entityName) const {
	auto uri = aliasToUri(alias);
	if (uri.has_value()) {
		return uri.value().get() + "#" + entityName;
	} else {
		return uri;
	}
}

std::vector<std::string_view> PrefixRegistry::getAliasesWithPrefix(const std::string &prefix) const {
	auto range_it = aliasToURI_.equal_range(PrefixProbe{prefix});
	std::vector<std::string_view> result;
	for (auto it = range_it.first; it != range_it.second; ++it) {
		result.emplace_back(it->first.c_str());
	}
	return result;
}
