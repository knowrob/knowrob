/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEFINED_PREDICATE_H_
#define KNOWROB_DEFINED_PREDICATE_H_

#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/plugins/NamedPlugin.h"

namespace knowrob {
	/**
	 * A predicate description with associated reasoners that can evaluate the predicate.
	 */
	class DefinedPredicate {
	public:
		/**
		 * @param indicator the indicator of the described predicate.
		 */
		explicit DefinedPredicate(const std::shared_ptr<PredicateIndicator> &indicator);

		/**
		 * Add a reasoner to this description.
		 * @param managedReasoner a managed reasoner.
		 * @param definition a predicate description.
		 * @return true if the reasoner was added successfully.
		 */
		bool addReasoner(const std::shared_ptr<NamedReasoner> &managedReasoner,
						 const std::shared_ptr<PredicateDescription> &definition);

		/**
		 * @return the type of the described predicate.
		 */
		PredicateType predicateType() const { return predicateType_; }

		/**
		 * @return set of reasoners associated to this description.
		 */
		const std::set<std::shared_ptr<NamedReasoner>> &reasonerEnsemble() const { return reasonerEnsemble_; }

	protected:
		const std::shared_ptr<PredicateIndicator> indicator_;
		PredicateType predicateType_;
		std::set<std::shared_ptr<NamedReasoner>> reasonerEnsemble_;
	};
}

#endif //KNOWROB_DEFINED_PREDICATE_H_
