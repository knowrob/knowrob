/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_BLACKBOARD_H__
#define __KNOWROB_BLACKBOARD_H__

// STD
#include <list>
#include <map>
#include <memory>
// KnowRob
#include <knowrob/qa/queries.h>
#include <knowrob/qa/BlackboardSegment.h>
#include <knowrob/reasoning/ReasonerManager.h>

namespace knowrob {
	/**
	 * A board where multiple experts can contribute in answering a query.
	 */
	class Blackboard {
	public:
		Blackboard(
			const std::shared_ptr<ReasonerManager> &reasonerManager,
			const std::shared_ptr<QueryResultQueue> &outputQueue,
			const std::shared_ptr<Query> &goal);
		// copy constructor is not supported for blackboards
		Blackboard(const Blackboard&) = delete;
		~Blackboard();

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<QueryResultQueue> outputQueue_;
		std::shared_ptr<Query> goal_;
		
		std::list<std::shared_ptr<BlackboardSegment>> segments_;

		/** Decompose the blackboard into different segments. */
		void decompose(
			const std::shared_ptr<Formula> &phi,
			std::shared_ptr<QueryResultQueue> &in,
			std::shared_ptr<QueryResultQueue> &out);
		
		void decomposePredicate(
			const std::shared_ptr<PredicateFormula> &phi,
			std::shared_ptr<QueryResultQueue> &in,
			std::shared_ptr<QueryResultQueue> &out);
		
		void decomposeConjunction(
			const std::shared_ptr<ConjunctionFormula> &phi,
			std::shared_ptr<QueryResultQueue> &in,
			std::shared_ptr<QueryResultQueue> &out);
		
		void decomposeDisjunction(
			const std::shared_ptr<DisjunctionFormula> &phi,
			std::shared_ptr<QueryResultQueue> &in,
			std::shared_ptr<QueryResultQueue> &out);

		/** Stop all reasoning processes attached to segments. */
		void stopReasoningProcesses();
	};
}

#endif //__KNOWROB_BLACKBOARD_H__
