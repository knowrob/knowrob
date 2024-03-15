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
	 * A flag that indicates whether a SPARQL feature is supported or not.
	 */
	enum class SPARQLFlag : std::uint8_t {
		NOTHING = 1ul << 0,
		NOT_EXISTS_UNSUPPORTED = 1ul << 1
	};
	using SPARQLFlags = SPARQLFlag;

	/**
	 * Compute the bitwise OR of two SPARQL flags.
	 * @param a a flag.
	 * @param b a flag.
	 * @return the bitwise OR of a and b.
	 */
	SPARQLFlag operator|(SPARQLFlag a, SPARQLFlag b);

	/**
	 * Compute the bitwise AND of two SPARQL flags.
	 * @param a a flag.
	 * @param b a flag.
	 * @return the bitwise AND of a and b.
	 */
	bool operator&(SPARQLFlag a, SPARQLFlag b);

	/**
	 * A SPARQL query using 1.1 syntax.
	 * But since some backends do not support all features, this class allows to
	 * specify some features that cannot be used.
	 */
	class SPARQLQuery {
	public:
		/**
		 * Create a SPARQL query that selects all triples matching the given pattern.
		 * @param triplePattern the pattern to match.
		 */
		explicit SPARQLQuery(const FramedTriplePattern &triplePattern, SPARQLFlags flags = SPARQLFlag::NOTHING);

		/**
		 * @param triplePatterns the patterns to match.
		 */
		explicit SPARQLQuery(const std::shared_ptr<GraphQuery> &query, SPARQLFlags flags = SPARQLFlag::NOTHING);

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
		SPARQLFlags flags_;
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
