/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SPARQL_SERVICE_H
#define KNOWROB_SPARQL_SERVICE_H

#include <redland.h>
#include "knowrob/db/DataService.h"
#include "knowrob/semweb/TripleFormat.h"
#include "OntologySource.h"
#include "knowrob/semweb/TripleContainer.h"

namespace knowrob {
	/**
	 * A SPARQL service is a data service that can be queried for triples.
	 */
	class SPARQLService : public DataService, public OntologySource {
	public:
		/**
		 * @param uri the URI of the SPARQL endpoint.
		 * @param format the format of the triples in the endpoint.
		 */
		SPARQLService(const URI &uri, semweb::TripleFormat format);

		/**
		 * @param uri the URI of the SPARQL endpoint.
		 * @param format the format of the triples in the endpoint.
		 */
		SPARQLService(const URI &uri, std::string_view format);

		~SPARQLService();

		/**
		 * @return the format of the triples in the file.
		 */
		semweb::TripleFormat tripleFormat() const { return tripleFormat_; }

		/**
		 * The meaning is that parent origin imports this ontology file.
		 * @param parentOrigin the origin of the parent ontology.
		 */
		void setParentOrigin(std::string_view parentOrigin) { parentOrigin_ = parentOrigin; }

		/**
		 * @return the origin of the parent ontology.
		 */
		auto &parentOrigin() const { return parentOrigin_; }

		/**
		 * @param callback a function that is called for each batch of triples in the endpoint.
		 * @param origin the origin for each triple.
		 * @param batchSize the number of triples to load in each batch.
		 * @return true if the triples were loaded successfully.
		 */
		bool load(const semweb::TripleHandler &callback, std::string_view origin, uint32_t batchSize);

	protected:
		semweb::TripleFormat tripleFormat_;
		std::optional<std::string> parentOrigin_;
	    librdf_world *world_;
	    librdf_storage *storage_;
		librdf_model *model_;

		bool createModel();
	};

} // knowrob

#endif //KNOWROB_SPARQL_SERVICE_H
