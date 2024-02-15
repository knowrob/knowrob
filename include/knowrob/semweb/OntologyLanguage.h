/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_LANGUAGE_H
#define KNOWROB_ONTOLOGY_LANGUAGE_H

#include "string_view"

namespace knowrob::semweb {
	/**
	 * Used to indicate the file format when loading triple data.
	 */
	enum OntologyLanguage {
		RDFS,
		OWL
	};

	OntologyLanguage ontologyLanguageFromString(std::string_view format);
	std::string_view ontologyLanguageToString(OntologyLanguage format);
	bool isOntologyLanguageString(std::string_view format);
}

#endif //KNOWROB_ONTOLOGY_LANGUAGE_H
