/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_QUERY_H
#define KNOWROB_GRAPH_QUERY_H

#include "knowrob/queries/Query.h"
#include "knowrob/triples/GraphTerm.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob {
	/**
	 * A Query that is constructed from a sequence of literals which are considered to be in a conjunction.
	 * The literals are part of a dependency group, meaning that they are connected through free variables.
	 */
	class GraphQuery : public Query {
	public:
		/**
		 * @param query an ordered sequence of graph terms.
		 * @param ctx the query context.
		 */
		explicit GraphQuery(std::shared_ptr<GraphTerm> &query, const QueryContextPtr &ctx = DefaultQueryContext());

		/**
		 * @param query an ordered sequence of triple patterns.
		 * @param ctx the query context.
		 */
		explicit GraphQuery(const std::vector<FramedTriplePatternPtr> &query,
							const QueryContextPtr &ctx = DefaultQueryContext());

		/**
		 * @param query a single triple pattern.
		 * @param ctx the query context.
		 */
		explicit GraphQuery(const FramedTriplePatternPtr &query,
							const QueryContextPtr &ctx = DefaultQueryContext());

		/**
		 * @return the GraphTerm's of the query, which are considered to be in a conjunction.
		 */
		auto term() const { return term_; }

		/**
		 * Generate a formula from the graph query.
		 * @return a formula that represents the query.
		 */
		FormulaPtr toFormula() const;

	protected:
		std::shared_ptr<GraphTerm> term_;

		void write(std::ostream &os) const override;

		explicit GraphQuery(const QueryContextPtr &ctx = DefaultQueryContext()) : Query(ctx) {}
	};

	// A shared pointer to a GraphQuery
	using GraphQueryPtr = std::shared_ptr<GraphQuery>;

} // knowrob

#endif //KNOWROB_GRAPH_QUERY_H
