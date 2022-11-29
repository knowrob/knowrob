/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONING_TASK_H__
#define __KNOWROB_REASONING_TASK_H__

// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/reasoning/IReasoner.h>
#include <knowrob/lang/Query.h>

namespace knowrob {
	/**
	 * The task of a reasoner to answer a certain query.
	 */
	class ReasoningTask {
	public:
		ReasoningTask(
			boost::shared_ptr<IReasoner> &reasoner,
			boost::shared_ptr<QueryResultQueue> &inputQueue,
			boost::shared_ptr<QueryResultQueue> &outputQueue,
			boost::shared_ptr<Query> &goal)
		: reasoner_(reasoner),
		  inputQueue_(inputQueue),
		  outputQueue_(outputQueue),
		  goal_(goal) {};

		/** Get the reasoner associated to this task.
		 *
		 * @return the reasoner associated to this task
		 */
		const boost::shared_ptr<IReasoner>& reasoner() const { return reasoner_; }

		/** Get the goal associated to this task.
 		 *
		 * @return the goal associated to this task
		 */
		const boost::shared_ptr<Query>& goal() const { return goal_; }
        
		/**
		 */
		const boost::shared_ptr<QueryResultQueue>& inputQueue() const { return inputQueue_; }
		
		/**
		 */
 		const boost::shared_ptr<QueryResultQueue>& outputQueue() const { return outputQueue_; }

	protected:
		boost::shared_ptr<IReasoner> reasoner_;
		boost::shared_ptr<QueryResultQueue> inputQueue_;
		boost::shared_ptr<QueryResultQueue> outputQueue_;
		boost::shared_ptr<Query> goal_;
	};
}

#endif //__KNOWROB_REASONING_TASK_H__
