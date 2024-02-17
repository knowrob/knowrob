/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <unordered_map>
#include "knowrob/semweb/OntologyLanguage.h"

static const std::unordered_map<std::string_view, knowrob::semweb::OntologyLanguage> stringToMap = {
	{"rdfs", knowrob::semweb::OntologyLanguage::RDFS},
	{"RDFS", knowrob::semweb::OntologyLanguage::RDFS},
	{"owl", knowrob::semweb::OntologyLanguage::OWL},
	{"OWL", knowrob::semweb::OntologyLanguage::OWL}
};

namespace knowrob::semweb {
	bool isOntologyLanguageString(std::string_view format) {
		auto it = stringToMap.find(format);
		return (it != stringToMap.end());
	}

	OntologyLanguage ontologyLanguageFromString(std::string_view format) {
		auto it = stringToMap.find(format);
		if (it != stringToMap.end()) {
			return it->second;
		}
		return OntologyLanguage::OWL;
	}

	std::string_view ontologyLanguageToString(OntologyLanguage format) {
		switch (format) {
			case OntologyLanguage::RDFS:
				return "RDFS";
			case OntologyLanguage::OWL:
				return "OWL";
		}
		return "OWL";
	}
}
