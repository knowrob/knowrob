/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SPARQL_BACKEND_H
#define KNOWROB_SPARQL_BACKEND_H

#include "knowrob/db/QueryableBackend.h"
#include "knowrob/triples/SPARQLQuery.h"

namespace knowrob {
	/**
	 * A backend that implements querying via SPARQL.
	 */
	class SPARQLBackend : public QueryableBackend {
	public:
		explicit SPARQLBackend(SPARQLFlags flags) : sparqlFlags_(flags) {}

		/**
		 * Run a SPARQL query on the model.
		 * @param queryString the query to run.
		 * @param callback the callback to handle the results.
		 * @return true if the query was successful.
		 */
		virtual bool sparql(std::string_view queryString, const BindingsHandler &callback) const = 0;

		/**
		 * Run a SPARQL query on the model.
		 * @param query the query to run.
		 * @param callback the callback to handle the results.
		 * @return true if the query was successful.
		 */
		bool query(const SPARQLQuery &query, const BindingsHandler &callback) const;

		// Override QueryableBackend
		void query(const GraphQueryPtr &query, const BindingsHandler &callback) override;

		// Override QueryableBackend
		void count(const ResourceCounter &callback) const override;

	protected:
		SPARQLFlags sparqlFlags_;
	};

} // knowrob

#endif //KNOWROB_SPARQL_BACKEND_H
