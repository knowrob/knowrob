/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>
#include "knowrob/sources/OntologyFile.h"

using namespace knowrob;

OntologyFile::OntologyFile(const URI &uri, std::string_view format)
		: DataFile(uri, format),
		  OntologySource(),
		  tripleFormat_(semweb::tripleFormatFromString(format)),
		  ontologyLanguage_(semweb::OntologyLanguage::OWL),
		  origin_(DataSource::getNameFromURI(uri())) {
}
