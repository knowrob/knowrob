/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERYABLE_BACKEND_H
#define KNOWROB_QUERYABLE_BACKEND_H

#include "knowrob/queries/ConjunctiveQuery.h"
#include "knowrob/queries/TokenBuffer.h"

namespace knowrob {
	/**
	 * A backend that can be queried with graph queries.
	 */
	class QueryableBackend {
	public:
		/**
		 * Submits a graph query to this knowledge graph.
		 * The query is evaluated concurrently, and evaluation may still be active
		 * when this function returns.
		 * The function returns a stream of solutions, the end of the stream is indicated
		 * by an EOS message.
		 * @param query a graph query
		 * @return a stream with answers to the query
		 */
		TokenBufferPtr submitQuery(const ConjunctiveQueryPtr &query);

		/**
		 * Evaluates a query and may block until evaluation completed.
		 * All results will be written into the provided stream object.
		 * @param query a query.
		 * @param resultStream a stream of answers.
		 */
		virtual void evaluateQuery(const ConjunctiveQueryPtr &query, const TokenBufferPtr &resultStream) = 0;
	};

	using QueryableBackendPtr = std::shared_ptr<QueryableBackend>;
}

#endif //KNOWROB_QUERYABLE_BACKEND_H
