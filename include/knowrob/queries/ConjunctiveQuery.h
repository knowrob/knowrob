/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONJUNCTIVE_QUERY_H
#define KNOWROB_CONJUNCTIVE_QUERY_H

#include "Query.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob {
	/**
	 * A Query that is constructed from a sequence of literals which are considered to be in a conjunction.
	 * The literals are part of a dependency group, meaning that they are connected through free variables.
	 */
	class ConjunctiveQuery : public Query {
	public:
		/**
		 * @param literals an ordered sequence of literals.
		 * @param ctx the query context.
		 */
		explicit ConjunctiveQuery(const std::vector<FramedTriplePatternPtr> &literals,
								  const QueryContextPtr &ctx = DefaultQueryContext());

		/**
		 * @param literal a single literal.
		 * @param ctx the query context.
		 */
		explicit ConjunctiveQuery(const FramedTriplePatternPtr &literal,
								  const QueryContextPtr &ctx = DefaultQueryContext());

		/**
		 * @return the literals of the query.
		 */
		const auto &literals() const { return literals_; }

	protected:
		std::vector<FramedTriplePatternPtr> literals_;
		FormulaPtr formula_;

		// Override Query
		void write(std::ostream &os) const override { os << *formula_; }

		explicit ConjunctiveQuery(QueryContextPtr ctx = DefaultQueryContext()) : Query(ctx) {}

		void init();
	};

	// A shared pointer to a GraphQuery
	using ConjunctiveQueryPtr = std::shared_ptr<ConjunctiveQuery>;

} // knowrob

#endif //KNOWROB_CONJUNCTIVE_QUERY_H
