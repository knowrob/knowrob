/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BUILTIN_EVALUATOR_H
#define KNOWROB_BUILTIN_EVALUATOR_H

#include "knowrob/reasoner/IReasoner.h"

namespace knowrob {
	/**
	 * A reasoner implementation that only evaluates builtin predicates.
	 * This is thought to be used to avoid calling a reasoner just for the evaluation
	 * of a builtin.
	 */
	class BuiltinEvaluator : public IReasoner {
	private:
		// it's a singleton
		BuiltinEvaluator();

	public:
		/**
		 * @return the BuiltinEvaluator singleton
		 */
		static const std::shared_ptr<BuiltinEvaluator>& get();

		/**
		 * @param indicator a predicate indicator
		 * @return true if the predicate is a supported builtin
		 */
		bool isBuiltinSupported(const std::shared_ptr<PredicateIndicator> &indicator);

		// Override IReasoner
        void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) override;

		// Override IReasoner
		unsigned long getCapabilities() const override;

		// Override IReasoner
		bool loadConfiguration(const ReasonerConfiguration &cfg) override;

		// Override IReasoner
		std::shared_ptr<PredicateDescription> getPredicateDescription(
				const std::shared_ptr<PredicateIndicator> &indicator) override;

		// Override IReasoner
        bool runQuery(const AllocatedQueryPtr &query) override;

	protected:
		static std::shared_ptr<BuiltinEvaluator> singleton_;
		// maps predicate indicator to builtin implementation
		std::map<PredicateIndicator, std::function<void(
				const AllocatedQueryPtr&,
				const std::vector<TermPtr>&)>> builtins_;

		static void pushSubstitution1(
				const AllocatedQueryPtr &queryInstance,
				Variable &var, const TermPtr& value);

		// builtin implementations
		void atom_concat3(const AllocatedQueryPtr &queryInstance, const std::vector<TermPtr> &args);
	};
}

#endif //KNOWROB_BUILTIN_EVALUATOR_H
