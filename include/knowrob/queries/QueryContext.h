/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_CONTEXT_H
#define KNOWROB_QUERY_CONTEXT_H

#include <utility>

#include "knowrob/modalities/ModalOperator.h"
#include "knowrob/semweb/GraphSelector.h"

namespace knowrob {
	/**
	 * The context in which a query is evaluated.
	 */
	struct QueryContext {
		explicit QueryContext(int queryFlags)
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
