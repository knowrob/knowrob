/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <unordered_map>
#include "knowrob/triples/TripleFormat.h"

static const std::unordered_map<std::string_view, knowrob::semweb::TripleFormat> stringToMap = {
	{"rdfxml", knowrob::semweb::TripleFormat::RDF_XML},
	{"rdf-xml", knowrob::semweb::TripleFormat::RDF_XML},
	{"rdf/xml", knowrob::semweb::TripleFormat::RDF_XML},
	{"RDFXML", knowrob::semweb::TripleFormat::RDF_XML},
	{"RDF-XML", knowrob::semweb::TripleFormat::RDF_XML},
	{"RDF/XML", knowrob::semweb::TripleFormat::RDF_XML},
	{"rdfa", knowrob::semweb::TripleFormat::RDFA},
	{"RDFA", knowrob::semweb::TripleFormat::RDFA},
	{"trig", knowrob::semweb::TripleFormat::TRIG},
	{"TRIG", knowrob::semweb::TripleFormat::TRIG},
	{"grddl", knowrob::semweb::TripleFormat::GRDDL},
	{"GRDDL", knowrob::semweb::TripleFormat::GRDDL},
	{"turtle", knowrob::semweb::TripleFormat::TURTLE},
	{"TURTLE", knowrob::semweb::TripleFormat::TURTLE},
	{"ntriples", knowrob::semweb::TripleFormat::N_TRIPLES},
	{"n-triples", knowrob::semweb::TripleFormat::N_TRIPLES},
	{"NTRIPLES", knowrob::semweb::TripleFormat::N_TRIPLES},
	{"N-TRIPLES", knowrob::semweb::TripleFormat::N_TRIPLES},
};
static const std::unordered_map<std::string_view, knowrob::semweb::TripleFormat> stringToMap2 = {
	{"xml", knowrob::semweb::TripleFormat::RDF_XML},
	{"XML", knowrob::semweb::TripleFormat::RDF_XML}
};

namespace knowrob::semweb {
	bool isTripleFormatString(std::string_view format) {
		auto it = stringToMap.find(format);
		return (it != stringToMap.end());
	}

	TripleFormat tripleFormatFromString(std::string_view format) {
		auto it = stringToMap.find(format);
		if (it != stringToMap.end()) {
			return it->second;
		}
		it = stringToMap2.find(format);
		if (it != stringToMap2.end()) {
			return it->second;
		}
		return TripleFormat::RDF_XML;
	}

	std::string_view tripleFormatToString(TripleFormat format) {
		switch (format) {
			case TripleFormat::RDF_XML:
				return "RDF/XML";
			case TripleFormat::RDFA:
				return "RDFA";
			case TripleFormat::TRIG:
				return "TRIG";
			case TripleFormat::GRDDL:
				return "GRDDL";
			case TripleFormat::TURTLE:
				return "TURTLE";
			case TripleFormat::N_TRIPLES:
				return "N-TRIPLES";
		}
		return "RDF/XML";
	}

	std::string_view tripleFormatMimeType(knowrob::semweb::TripleFormat format) {
		switch (format) {
			case semweb::RDF_XML:
				return "rdfxml";
			case semweb::TURTLE:
				return "turtle";
			case semweb::N_TRIPLES:
				return "ntriples";
			case semweb::RDFA:
				return "rdfa";
			case semweb::TRIG:
				return "trig";
			case semweb::GRDDL:
				return "grddl";
		}
		return "rdfxml";
	}
}
