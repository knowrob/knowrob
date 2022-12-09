/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONER_MANAGER_H__
#define __KNOWROB_REASONER_MANAGER_H__

// STD
#include <list>
#include <memory>
// KnowRob
#include <knowrob/knowrob.h>
#include <knowrob/lang/terms.h>
#include <knowrob/reasoning/IReasoner.h>

namespace knowrob {
	/**
	 * Manages a set of available reasoning subsystems.
	 */
	class ReasonerManager {
	public:
		ReasonerManager(uint32_t numInitialThreads, uint32_t maxNumThreads=0);

		/** Add a reasoner to this manager.
		 * @reasoner a reasoner.
		 */
		void addReasoner(const std::shared_ptr<IReasoner> &reasoner);

		/** Remove a reasoner from this manager.
		 * @reasoner a reasoner.
		 */
		void removeReasoner(const std::shared_ptr<IReasoner> &reasoner);

		/** Get list of reasoner that can handle given predicate.
		 *
		 * @param predicate the predicate in question
		 * @return an essemble of reasoner that can handle the predicate
		 */
		std::list<std::shared_ptr<IReasoner>> getReasonerForPredicate(const PredicateIndicator &predicate);
		
		/** Pushes a goal for a worker.
		 * The goal is assigned to a worker thread when one is available.
		 * @goal the work goal
		 */
		void pushGoal(const std::shared_ptr<IRunner> &goal);

	private:
		ThreadPool threadPool_;
		std::list<std::shared_ptr<IReasoner>> reasonerPool_;
	};
}

#endif //__KNOWROB_REASONER_MANAGER_H__
