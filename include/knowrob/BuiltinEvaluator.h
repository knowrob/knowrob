/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BUILTINEVALUATOR_H
#define KNOWROB_BUILTINEVALUATOR_H

#include <knowrob/IReasoner.h>

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
		unsigned long getCapabilities() const override;

		// Override IReasoner
		bool loadConfiguration(const ReasonerConfiguration &cfg) override;

		// Override IReasoner
		std::shared_ptr<PredicateDescription> getPredicateDescription(
				const std::shared_ptr<PredicateIndicator> &indicator) override;

		// Override IReasoner
		void startQuery(uint32_t queryID, const std::shared_ptr<const Query> &uninstantiatedQuery) override;

		// Override IReasoner
		void runQueryInstance(uint32_t queryID, const QueryInstancePtr &queryInstance) override;

		// Override IReasoner
		void finishQuery(uint32_t queryID,
						 const std::shared_ptr<QueryResultStream::Channel> &outputStream,
						 bool isImmediateStopRequested) override;
	protected:
		static std::shared_ptr<BuiltinEvaluator> singleton_;
		// maps predicate indicator to builtin implementation
		std::map<PredicateIndicator, std::function<void(
				const QueryInstancePtr&,
				const std::vector<TermPtr>&)>> builtins_;

		static void pushSubstitution1(
				const QueryInstancePtr &queryInstance,
				Variable &var, const TermPtr& value);

		// builtin implementations
		void atom_concat3(const QueryInstancePtr &queryInstance, const std::vector<TermPtr> &args);
	};
}

#endif //KNOWROB_BUILTINEVALUATOR_H
