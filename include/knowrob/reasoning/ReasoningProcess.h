/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONING_PROCESS_H__
#define __KNOWROB_REASONING_PROCESS_H__

#include <knowrob/reasoning/ReasoningTask.h>

namespace knowrob {
	/**
	 * A process that executes a reasoning task.
	 */
	class ReasoningProcess : public IRunner {
	public:
		ReasoningProcess(const ReasoningTask &task);

		/** Get the task of this process.
		 *
		 * @return the task
		 */
		const ReasoningTask& task() const { return task_; }

		/** Get the status of this process.
		 *
		 * @return the status
		 */
		const ReasoningStatus& status() const { return status_; }
		
		/** Stop this reasoning process.
		 * @wait if true the call will block until stopped.
		 */
		void stop(bool wait=false);
		
		// Override IRunner:run
		void run();

	private:
		ReasoningTask task_;
		bool isRunning_;
		bool hasStopRequest_;
		std::mutex mutex_;
		std::condition_variable finishedCV_;
		
	};
}

#endif //__KNOWROB_REASONING_PROCESS_H__
