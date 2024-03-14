/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_H_
#define KNOWROB_QUERY_H_

#include <ostream>
#include <knowrob/formulas/Formula.h>
#include "QueryContext.h"

namespace knowrob {
	QueryContextPtr DefaultQueryContext();

	QueryContextPtr OneSolutionContext();

	/**
	 * A baseclass for queries. The only commitment is that queries are evaluated
	 * within a certain context. The context defines additional parameters for the evaluation.
	 */
	class Query {
	public:
		/**
		 * @param ctx the query context.
		 */
		explicit Query(QueryContextPtr ctx = DefaultQueryContext()) : ctx_(std::move(ctx)) {}

		/**
		 * @return the query context.
		 */
		auto &ctx() const { return ctx_; }

		/**
		 * @param ctx the query context.
		 */
		void setContext(QueryContextPtr ctx) { ctx_ = std::move(ctx); }

	protected:
		QueryContextPtr ctx_;

		virtual void write(std::ostream &os) const = 0;

		friend struct QueryWriter;
	};

	/**
	 * Writes a term into an ostream.
	 */
	struct QueryWriter {
		QueryWriter(const Query &q, std::ostream &os) { q.write(os); }
	};
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Query &q);
}

#endif //KNOWROB_QUERY_H_
