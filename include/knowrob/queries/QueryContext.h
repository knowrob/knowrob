//
// Created by daniel on 17.01.24.
//

#ifndef KNOWROB_QUERY_CONTEXT_H
#define KNOWROB_QUERY_CONTEXT_H

#include <utility>

#include "knowrob/modalities/ModalOperator.h"
#include "knowrob/semweb/GraphSelector.h"

namespace knowrob {
	class QueryContext {
	public:
		explicit QueryContext(int queryFlags)
		: queryFlags_(queryFlags), modalIteration_(ModalIteration::emptyIteration())
		{}

		QueryContext(const QueryContext &other, const ModalOperatorPtr &modalOperator)
		: queryFlags_(other.queryFlags_), modalIteration_(other.modalIteration_ + modalOperator)
		{
		}

		int queryFlags_;

		ModalIteration modalIteration_;

		GraphSelector selector_;
	};

    using QueryContextPtr = std::shared_ptr<const QueryContext>;
};

#endif //KNOWROB_EVALUATION_CONTEXT_H
