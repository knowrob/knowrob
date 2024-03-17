/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_FILE_H
#define KNOWROB_ONTOLOGY_FILE_H

#include "knowrob/sources/DataFile.h"
#include "knowrob/sources/OntologySource.h"
#include "knowrob/triples/TripleContainer.h"
#include "knowrob/triples/TripleFormat.h"
#include "knowrob/semweb/OntologyLanguage.h"

namespace knowrob {
	/**
	 * An ontology file is a data source that provides ontology data in a file.
	 */
	class OntologyFile : public DataFile, public OntologySource {
	public:
		/**
		 * @param uri URI of the data source.
		 * @param format string identifier of the data format.
		 */
		OntologyFile(const URI &uri, std::string_view format);

		/**
		 * @return the format of the triples in the file.
		 */
		semweb::TripleFormat tripleFormat() const { return tripleFormat_; }

		/**
		 * @param language the language of the ontology.
		 */
		void setOntologyLanguage(semweb::OntologyLanguage language) { ontologyLanguage_ = language; }

		/**
		 * @return the language of the ontology.
		 */
		semweb::OntologyLanguage ontologyLanguage() const { return ontologyLanguage_; }

		// override DataSource
		DataSourceType type() const override { return DataSourceType::ONTOLOGY; }

		// override OntologySource
		std::string_view origin() const override { return origin_; }

	protected:
		semweb::TripleFormat tripleFormat_;
		semweb::OntologyLanguage ontologyLanguage_;
		std::optional<std::string> parentOrigin_;
		std::string origin_;
	};

} // knowrob

#endif //KNOWROB_ONTOLOGY_FILE_H
