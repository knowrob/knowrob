/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONING_TASK_H__
#define __KNOWROB_REASONING_TASK_H__

// STD
#include <memory>
// KnowRob
#include <knowrob/reasoning/IReasoner.h>
#include <knowrob/qa/queries.h>

namespace knowrob {
	/**
	 * The task of a reasoner to answer a certain query.
	 */
	class ReasoningTask {
	public:
		ReasoningTask(
			const std::shared_ptr<IReasoner> &reasoner,
			const std::shared_ptr<QueryResultQueue> &inputQueue,
			const std::shared_ptr<QueryResultQueue> &outputQueue,
			const std::shared_ptr<Query> &goal)
		: reasoner_(reasoner),
		  inputQueue_(inputQueue),
		  outputQueue_(outputQueue),
		  goal_(goal) {};

		/** Get the reasoner associated to this task.
		 *
		 * @return the reasoner associated to this task
		 */
		const std::shared_ptr<IReasoner>& reasoner() const { return reasoner_; }

		/** Get the goal associated to this task.
 		 *
		 * @return the goal associated to this task
		 */
		const std::shared_ptr<Query>& goal() const { return goal_; }
        
		/**
		 */
		const std::shared_ptr<QueryResultQueue>& inputQueue() const { return inputQueue_; }
		
		/**
		 */
 		const std::shared_ptr<QueryResultQueue>& outputQueue() const { return outputQueue_; }

	protected:
		std::shared_ptr<IReasoner> reasoner_;
		std::shared_ptr<QueryResultQueue> inputQueue_;
		std::shared_ptr<QueryResultQueue> outputQueue_;
		std::shared_ptr<Query> goal_;
	};
}

#endif //__KNOWROB_REASONING_TASK_H__
