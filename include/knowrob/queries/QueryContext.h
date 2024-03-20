/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_CONTEXT_H
#define KNOWROB_QUERY_CONTEXT_H

#include <utility>

#include "knowrob/formulas/ModalOperator.h"
#include "knowrob/triples/GraphSelector.h"

namespace knowrob {
	enum QueryFlag {
		QUERY_FLAG_ALL_SOLUTIONS = 1 << 0,
		QUERY_FLAG_ONE_SOLUTION = 1 << 1,
		QUERY_FLAG_PERSIST_SOLUTIONS = 1 << 2,
		QUERY_FLAG_UNIQUE_SOLUTIONS = 1 << 3,
		//QUERY_FLAG_ORDER_PRESERVING = 1 << 4,
	};

	/**
	 * The context in which a query is evaluated.
	 */
	struct QueryContext {
		explicit QueryContext(int queryFlags = QUERY_FLAG_ALL_SOLUTIONS)
				: queryFlags(queryFlags), modalIteration(ModalIteration::emptyIteration()) {}

		QueryContext(const QueryContext &other, const ModalOperatorPtr &modalOperator)
				: queryFlags(other.queryFlags), modalIteration(other.modalIteration + modalOperator) {
		}

		/**
		 * The query flags bitmask with values from QueryFlags.
		 */
		int queryFlags;
		/**
		 * The modal iteration in which the query is evaluated.
		 */
		ModalIteration modalIteration;
		/**
		 * The frame of triples in this context.
		 */
		GraphSelector selector;
	};

	using QueryContextPtr = std::shared_ptr<const QueryContext>;
};

#endif //KNOWROB_QUERY_CONTEXT_H
