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
		FramedPredicate(PredicatePtr predicate, GraphSelectorPtr graphSelector,
				std::shared_ptr<const StringTerm> reasonerTerm)
				: predicate_(std::move(predicate)),
				  graphSelector_(std::move(graphSelector)),
				  reasonerTerm_(std::move(reasonerTerm)) {}

		/**
		 * @return the predicate.
		 */
		auto &predicate() const { return predicate_; }

		/**
		 * @return the graph selector.
		 */
		auto &graphSelector() const { return graphSelector_; }

		/**
		 * @return the reasoner term.
		 */
		auto &reasonerTerm() const { return reasonerTerm_; }

	protected:
		PredicatePtr predicate_;
		GraphSelectorPtr graphSelector_;
		std::shared_ptr<const StringTerm> reasonerTerm_;
	};
}

#endif //KNOWROB_PREDICATE_DESCRIPTION_H
