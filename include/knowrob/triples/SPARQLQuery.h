/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SPARQL_QUERY_H
#define KNOWROB_SPARQL_QUERY_H

#include "knowrob/triples/FramedTriple.h"
#include "knowrob/triples/FramedTriplePattern.h"
#include "GraphQuery.h"

namespace knowrob {
	/**
	 * A SPARQL query.
	 */
	class SPARQLQuery {
	public:
		/**
		 * Create a SPARQL query that selects all triples matching the given pattern.
		 * @param triplePattern the pattern to match.
		 */
		explicit SPARQLQuery(const FramedTriplePattern &triplePattern);

		/**
		 * @param triplePatterns the patterns to match.
		 */
		explicit SPARQLQuery(const std::shared_ptr<GraphQuery> &query);

		/**
		 * @return the query string.
		 */
		std::string_view operator()() const { return queryString_; }

		/**
		 * @return the query string as a unsigned C string.
		 */
		const unsigned char *
		asUnsignedString() const { return reinterpret_cast<const unsigned char *>(queryString_.c_str()); }

	protected:
		uint32_t varCounter_;
		std::string queryString_;

		static void selectBegin(std::ostream &os);

		static void selectEnd(std::ostream &os);

		void add(std::ostream &os, const FramedTriplePattern &triplePattern);

		void add(std::ostream &os, const std::shared_ptr<GraphTerm> &graphTerm);

		void filterNotExists(std::ostream &os, const FramedTriplePattern &triplePattern);

		void optional(std::ostream &os, const FramedTriplePattern &triplePattern);

		void where(std::ostream &os, const FramedTriplePattern &triplePattern);

		void where(std::ostream &os, const TermPtr &term);

		static void filter(std::ostream &os, std::string_view varName, const TermPtr &term,
						   FramedTriplePattern::OperatorType operatorType);
	};

} // knowrob

#endif //KNOWROB_SPARQL_QUERY_H
