/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SPARQL_QUERY_H
#define KNOWROB_SPARQL_QUERY_H

#include "knowrob/triples/FramedTriple.h"
#include "knowrob/triples/FramedTriplePattern.h"
#include "GraphQuery.h"
#include "GraphBuiltin.h"

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
		std::map<std::string_view, std::string_view> aliases_;
		std::string queryString_;
		std::string lastVar_;

		static void selectBegin(std::ostream &os);

		static void selectEnd(std::ostream &os);

		void appendPrefixes(std::ostream &os);

		void add(std::ostream &os, const FramedTriplePattern &triplePattern);

		void add(std::ostream &os, const GraphBuiltin &builtin);

		void add(std::ostream &os, const std::shared_ptr<GraphTerm> &graphTerm);

		bool optional(std::ostream &os, const FramedTriplePattern &triplePattern);

		void iri(std::ostream &os, std::string_view iri);

		void negationViaNotExists(std::ostream &os, const FramedTriplePattern &triplePattern);

		void negationViaOptional(std::ostream &os, const FramedTriplePattern &triplePattern);

		void negationViaMinus(std::ostream &os, const FramedTriplePattern &triplePattern);

		bool where(std::ostream &os, const FramedTriplePattern &triplePattern);

		void where_with_filter(std::ostream &os, const FramedTriplePattern &triplePattern);

		void where(std::ostream &os, const TermPtr &term);

		void comparison(std::ostream &os, const GraphBuiltin &builtin, const char *comparisonOperator);

		void bindOneOfIf(std::ostream &os, const GraphBuiltin &builtin, const char *comparisonOperator);

		void filter(std::ostream &os, std::string_view varName, const TermPtr &term,
						   FramedTriplePattern::OperatorType operatorType);

		void filter_optional(std::ostream &os, std::string_view varName, const TermPtr &term,
									FramedTriplePattern::OperatorType operatorType);

		void doFilter(std::ostream &os, std::string_view varName, const std::shared_ptr<Atomic> &atomic,
							 FramedTriplePattern::OperatorType operatorType);
	};

} // knowrob

#endif //KNOWROB_SPARQL_QUERY_H
