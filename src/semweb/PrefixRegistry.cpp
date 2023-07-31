//
// Created by daniel on 11.02.23.
//

#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/semweb/PrefixProbe.h"

using namespace knowrob::semweb;

PrefixRegistry::PrefixRegistry()
{
    registerPrefix("owl",  "http://www.w3.org/2002/07/owl");
    registerPrefix("rdf",  "http://www.w3.org/1999/02/22-rdf-syntax-ns");
    registerPrefix("rdfs", "http://www.w3.org/2000/01/rdf-schema");
    registerPrefix("xsd",  "http://www.w3.org/2001/XMLSchema");
    registerPrefix("dul",  "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl");
}

PrefixRegistry& PrefixRegistry::get()
{
    static PrefixRegistry singleton;
    return singleton;
}

void PrefixRegistry::registerPrefix(const std::string &prefix, const std::string &uri)
{
    if(uri[uri.size()-1]=='#') {
        auto x = uri;
        x.pop_back();
        uriToAlias_[x] = prefix;
        aliasToURI_[prefix] = x;
    }
    else {
        uriToAlias_[uri] = prefix;
        aliasToURI_[prefix] = uri;
    }
}

OptionalStringRef PrefixRegistry::uriToAlias(const std::string &uri) const
{
    if(uri[uri.size()-1]=='#') {
        auto x = uri;
        x.pop_back();
        auto it = uriToAlias_.find(x);
        return it == uriToAlias_.end() ? std::nullopt : OptionalStringRef(it->second);
    }
    else {
        auto it = uriToAlias_.find(uri);
        return it == uriToAlias_.end() ? std::nullopt : OptionalStringRef(it->second);
    }
}

OptionalStringRef PrefixRegistry::aliasToUri(const std::string &alias) const
{
    auto it = aliasToURI_.find(alias);
    return it == aliasToURI_.end() ? std::nullopt : OptionalStringRef(it->second);
}

std::optional<std::string> PrefixRegistry::createIRI(const std::string &alias, const std::string &entityName) const
{
    auto uri = aliasToUri(alias);
    if(uri.has_value()) {
        return uri.value().get() + "#" + entityName;
    }
    else {
        return uri;
    }
}

std::vector<std::string_view> PrefixRegistry::getAliasesWithPrefix(const std::string &prefix) const
{
    auto range_it = aliasToURI_.equal_range(PrefixProbe { prefix });
    std::vector<std::string_view> result;
    for (auto it = range_it.first; it != range_it.second; ++it) {
        result.emplace_back(it->first.c_str());
    }
    return result;
}
