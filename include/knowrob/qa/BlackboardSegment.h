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
#include <memory>
// KnowRob
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/ReasonerManager.h>
#include <knowrob/reasoning/ReasoningTask.h>

namespace knowrob {
	
	/**
	 * The segment of a blackboard where an essemble of experts is working
	 * on solving a common task.
	 */
	class BlackboardSegment {
	public:
		BlackboardSegment(
			const std::shared_ptr<ReasonerManager> &reasonerManager,
			const ReasoningTask &tsk);
		
		~BlackboardSegment();
		
		// copy constructor is not supported for blackboards
		BlackboardSegment(const BlackboardSegment&) = delete;

		/** Start all reasoning processes attached to this segment.
		 */
		void start();
		
		/** Stop all reasoning processes attached to this segment.
		 */
		void stop(bool wait=false);
		
		/**
		 * A process that executes a reasoning task.
		 */
		class Runner : public IRunner {
		public:
			Runner(const ReasoningTask &task);
			
			// Override IRunner
			void stop(bool wait);
			// Override IRunner
			void run();

		private:
			ReasoningTask task_;
		};

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<BlackboardSegment::Runner> process_;
		ReasoningTask task_;
	};
}

#endif //__KNOWROB_BLACKBOARD_SEGMENT_H__
