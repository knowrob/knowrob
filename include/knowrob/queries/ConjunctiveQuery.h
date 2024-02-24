/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONJUNCTIVE_QUERY_H
#define KNOWROB_CONJUNCTIVE_QUERY_H

#include "Query.h"
#include "knowrob/semweb/FramedTriplePattern.h"

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
		ConjunctiveQuery(const std::vector<FramedTriplePatternPtr> &literals, const QueryContextPtr &ctx);

		/**
		 * @param literal a single literal.
		 * @param ctx the query context.
		 */
		ConjunctiveQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx);

		/**
		 * @return the literals of the query.
		 */
		const auto &literals() const { return literals_; }

		// Override Query
		const FormulaPtr &formula() const override;

		// Override Query
		std::ostream &print(std::ostream &os) const override;

		// Override Query
		QueryType type() const override { return QueryType::CONJUNCTIVE; }

	protected:
		std::vector<FramedTriplePatternPtr> literals_;
		FormulaPtr formula_;

		void init();
	};

	// A shared pointer to a GraphQuery
	using ConjunctiveQueryPtr = std::shared_ptr<ConjunctiveQuery>;

} // knowrob

#endif //KNOWROB_CONJUNCTIVE_QUERY_H
