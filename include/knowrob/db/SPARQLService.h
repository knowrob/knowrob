/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SPARQL_SERVICE_H
#define KNOWROB_SPARQL_SERVICE_H

#include <redland.h>
#include "knowrob/db/DataService.h"
#include "knowrob/triples/TripleFormat.h"
#include "knowrob/triples/TripleContainer.h"
#include "OntologySource.h"
#include "RedlandModel.h"

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

		/**s
		 * @param uri the URI of the SPARQL endpoint.
		 * @param format the format of the triples in the endpoint.
		 */
		SPARQLService(const URI &uri, std::string_view format);

		/**
		 * Load triples from the SPARQL endpoint.
		 * @param callback the callback to handle the triples.
		 * @return true if the triples were loaded successfully.
		 */
		bool load(const semweb::TripleHandler &callback);

		// Override OntologySource
		std::string_view origin() const override { return origin_; }

	protected:
		RedlandModel model_;
		std::string origin_;
	};

} // knowrob

#endif //KNOWROB_SPARQL_SERVICE_H
