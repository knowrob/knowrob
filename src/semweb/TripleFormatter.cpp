/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>
#include "knowrob/semweb/TripleFormatter.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/Logger.h"

bool knowrob::semweb::TripleFormatter::exportTo(
        const std::map<std::string_view, TriplePtr> &triples,
        const std::string &filename,
        TripleFormat format) {
    switch (format) {
        case TripleFormat::RDF_XML:
            return exportRDF_XML(triples, filename);
        default:
            // Unsupported format
            KB_WARN("Unsupported format: {}", tripleFormatToString(format));
            return false;
    }
}

bool knowrob::semweb::TripleFormatter::exportRDF_XML(
        const std::map<std::string_view, TriplePtr> &triples,
        const std::string &filename) {
    // make sure the directory exists
    std::filesystem::path path(filename);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    // open file for writing, overwrite if it exists
    std::ofstream file(filename);
    if (!file.is_open()) {
        KB_WARN("Could not open file {} for writing.", filename);
        return false;
    }
    // write RDF/XML header
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"\n";
    file << "         xmlns:rdfs=\"http://www.w3.org/2000/01/rdf-schema#\"\n";
    file << "         xmlns:owl=\"http://www.w3.org/2002/07/owl#\"\n";
    // iterate over known namespaces
    for (const auto &[uri, prefix] : PrefixRegistry::get()) {
        file << "         xmlns:" << prefix << "=\"" << uri << "\"\n";
    }
    file << ">\n";

    // write triples
    for (const auto &[subject,triple] : triples) {
        file << "  <rdf:Description rdf:about=\"" << subject << "\">\n";
        auto property = triple->predicate();
        auto valueString = triple->createStringValue();
        auto valueType = triple->xsdType();
        if (triple->isXSDLiteral()) {
            // XSD property assertion
            file << "    <" << property << " rdf:datatype=\"" << xsdTypeToIRI(valueType.value()) << "\">";
            file << valueString << "</" << property << ">\n";
        } else if (triple->isObjectIRI()) {
            // object property assertion
            file << "    <" << property << " rdf:resource=\"" << valueString << "\"/>\n";
        } else {
            // untyped literal
            file << "    <" << property << ">" << valueString << "</" << property << ">\n";
        }
        file << "  </rdf:Description>\n";
    }
    // write RDF/XML footer
    file << "</rdf:RDF>\n";
    file.close();
    return true;
}
