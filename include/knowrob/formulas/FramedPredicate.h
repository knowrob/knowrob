/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FRAMED_PREDICATE_H
#define KNOWROB_FRAMED_PREDICATE_H

#include "Predicate.h"

#include <utility>
#include "knowrob/semweb/GraphSelector.h"

namespace knowrob {
	class FramedPredicate {
	public:
		/**
		 * Create a framed predicate.
		 * @param predicate the predicate.
		 * @param graphSelector the graph selector.
		 */
		FramedPredicate(PredicatePtr predicate, GraphSelectorPtr graphSelector)
				: predicate_(std::move(predicate)), graphSelector_(std::move(graphSelector)) {}

		/**
		 * @return the predicate.
		 */
		const PredicatePtr &predicate() const { return predicate_; }

		/**
		 * @return the graph selector.
		 */
		const GraphSelectorPtr &graphSelector() const { return graphSelector_; }

	protected:
		PredicatePtr predicate_;
		GraphSelectorPtr graphSelector_;
	};
}

#endif //KNOWROB_PREDICATE_DESCRIPTION_H
