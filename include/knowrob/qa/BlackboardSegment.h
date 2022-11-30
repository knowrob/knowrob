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
#include <mutex>
#include <memory>
// KnowRob
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/ReasonerManager.h>

namespace knowrob {
	/**
	 * The segment of a blackboard where an essemble of experts is working
	 * on solving a common task.
	 */
	class BlackboardSegment {
	public:
		BlackboardSegment(
			const std::shared_ptr<ReasonerManager> &reasonerManager,
			const std::shared_ptr<QueryResultQueue> &inputQueue,
			const std::shared_ptr<QueryResultQueue> &outputQueue,
			const std::shared_ptr<Query> &query);
		// copy constructor is not supported for blackboards
		BlackboardSegment(const BlackboardSegment&) = delete;
		~BlackboardSegment();
		
		void addReasoner(const std::shared_ptr<IReasoner> &reasoner);
		
		void removeReasoner(const std::shared_ptr<IReasoner> &reasoner);

		/** Start all reasoning processes attached to this segment. */
		void startReasoningProcesses();
		
		/** Stop all reasoning processes attached to this segment. */
		void stopReasoningProcesses();

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<QueryResultQueue> inputQueue_;
		std::shared_ptr<QueryResultQueue> outputQueue_;
		std::shared_ptr<Query> goal_;
        	std::mutex mutex_;
        	bool isRunning_;
		
		std::list<std::shared_ptr<IReasoner>> reasoner_;
		std::map<std::shared_ptr<IReasoner>, std::shared_ptr<ReasoningProcess>> processes_;
		
		void startReasoner(const std::shared_ptr<IReasoner> &reasoner);
		void stopReasoner(const std::shared_ptr<IReasoner> &reasoner);
	};
}

#endif //__KNOWROB_BLACKBOARD_SEGMENT_H__
