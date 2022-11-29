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
// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/ReasonerManager.h>

namespace knowrob {
	/**
	 * A board where multiple experts can contribute in answering a query.
	 */
	class Blackboard {
	public:
		Blackboard(
			const boost::shared_ptr<ReasonerManager> &reasonerManager,
			const boost::shared_ptr<QueryResultQueue> &outputQueue,
			const boost::shared_ptr<Query> &goal);
		// copy constructor is not supported for blackboards
		Blackboard(const Blackboard&) = delete;
		~Blackboard();

	protected:
		boost::shared_ptr<ReasonerManager> reasonerManager_;,
		boost::shared_ptr<QueryResultQueue> outputQueue_;
		boost::shared_ptr<Query> goal_;
		
		std::list<boost::shared_ptr<BlackboardSegment>> segments_;

		/** Decompose the blackboard into different segments. */
		void decompose(const Formula &phi,
			boost::shared_ptr<QueryResultQueue> &in,
			boost::shared_ptr<QueryResultQueue> &out);
		
		void decomposePredicate(const PredicateFormula &phi,
			boost::shared_ptr<QueryResultQueue> &in,
			boost::shared_ptr<QueryResultQueue> &out);
		
		void decomposeConjunction(const ConjunctionFormula &phi,
			boost::shared_ptr<QueryResultQueue> &in,
			boost::shared_ptr<QueryResultQueue> &out);
		
		void decomposeDisjunction(const DisjunctionFormula &phi,
			boost::shared_ptr<QueryResultQueue> &in,
			boost::shared_ptr<QueryResultQueue> &out);

		/** Stop all reasoning processes attached to segments. */
		void stopReasoningProcesses();
	};
}

#endif //__KNOWROB_BLACKBOARD_H__
