//
// Created by daniel on 11.02.23.
//

#include "knowrob/rdf/PrefixRegistry.h"

using namespace knowrob;

rdf::PrefixRegistry::PrefixRegistry()
{
    registerPrefix("owl",  "http://www.w3.org/2002/07/owl");
    registerPrefix("rdf",  "http://www.w3.org/1999/02/22-rdf-syntax-ns");
    registerPrefix("rdfs", "http://www.w3.org/2000/01/rdf-schema");
    registerPrefix("xsd",  "http://www.w3.org/2001/XMLSchema");
}

rdf::PrefixRegistry& rdf::PrefixRegistry::get()
{
    static PrefixRegistry singleton;
    return singleton;
}

void rdf::PrefixRegistry::registerPrefix(const std::string &prefix, const std::string &uri)
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

rdf::OptionalStringRef rdf::PrefixRegistry::uriToAlias(const std::string &uri)
{
    if(uri[uri.size()-1]=='#') {
        auto x = uri;
        x.pop_back();
        auto it = uriToAlias_.find(x);
        return it == uriToAlias_.end() ? std::nullopt : rdf::OptionalStringRef(it->second);
    }
    else {
        auto it = uriToAlias_.find(uri);
        return it == uriToAlias_.end() ? std::nullopt : rdf::OptionalStringRef(it->second);
    }
}

rdf::OptionalStringRef rdf::PrefixRegistry::aliasToUri(const std::string &alias)
{
    auto it = aliasToURI_.find(alias);
    return it == aliasToURI_.end() ? std::nullopt : rdf::OptionalStringRef(it->second);
}
