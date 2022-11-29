/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_BLACKBOARD_SEGMENT_H__
#define __KNOWROB_BLACKBOARD_SEGMENT_H__

// STD
#include <list>
#include <map>
// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/lang/Query.h>
#include <knowrob/lang/QueryResult.h>
#include <knowrob/reasoning/ReasonerManager.h>

namespace knowrob {
	/**
	 * The segment of a blackboard where an essemble of experts is working
	 * on solving a common task.
	 */
	class BlackboardSegment {
	public:
		BlackboardSegment(
			const boost::shared_ptr<ReasonerManager> &reasonerManager,
			const boost::shared_ptr<QueryResultQueue> &inputQueue,
			const boost::shared_ptr<QueryResultQueue> &outputQueue,
			const boost::shared_ptr<Query> &query);
		// copy constructor is not supported for blackboards
		BlackboardSegment(const BlackboardSegment&) = delete;
		~BlackboardSegment();
		
		void addReasoner(boost::shared_ptr<IReasoner> &reasoner);
		
		void removeReasoner(boost::shared_ptr<IReasoner> &reasoner);

		/** Start all reasoning processes attached to this segment. */
		void startReasoningProcesses();
		
		/** Stop all reasoning processes attached to this segment. */
		void stopReasoningProcesses();

	protected:
		boost::shared_ptr<ReasonerManager> reasonerManager_;
		boost::shared_ptr<QueryResultQueue> inputQueue_;
		boost::shared_ptr<QueryResultQueue> outputQueue_;
		boost::shared_ptr<Query> goal_;
        	std::mutex mutex_;
		
		std::list<boost::shared_ptr<IReasoner>> reasoner_;
		std::map<boost::shared_ptr<IReasoner>, boost::shared_ptr<ReasoningProcess>> processes_;
		
		void startReasoner(boost::shared_ptr<IReasoner> &reasoner);
		void stopReasoner(boost::shared_ptr<IReasoner> &reasoner);
	};
}

#endif //__KNOWROB_BLACKBOARD_SEGMENT_H__
