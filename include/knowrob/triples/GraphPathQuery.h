/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_PATH_QUERY_H
#define KNOWROB_GRAPH_PATH_QUERY_H

#include "knowrob/triples/GraphQuery.h"

namespace knowrob {
	/**
	 * A GraphQuery that is constructed from a sequence of triple patterns which are considered to be in a conjunction.
	 */
	class GraphPathQuery : public GraphQuery {
	public:
		/**
		 * @param query an ordered sequence of triple patterns.
		 * @param ctx the query context.
		 */
		explicit GraphPathQuery(const std::vector<FramedTriplePatternPtr> &query,
								const QueryContextPtr &ctx = DefaultQueryContext())
				: GraphQuery(query, ctx), path_(query) {}

		/**
		 * @param query a single triple pattern.
		 * @param ctx the query context.
		 */
		explicit GraphPathQuery(const FramedTriplePatternPtr &query,
								const QueryContextPtr &ctx = DefaultQueryContext())
				: GraphQuery(query, ctx), path_({query}) {}

		/**
		 * @return the path of the query.
		 */
		auto &path() const { return path_; }

	protected:
		std::vector<FramedTriplePatternPtr> path_;
	};

	// A shared pointer to a GraphQuery
	using GraphPathQueryPtr = std::shared_ptr<GraphPathQuery>;
} // knowrob

#endif //KNOWROB_GRAPH_PATH_QUERY_H
