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
	 * A process bound to a thread where a particular reasoning task is executed.
	 */
	class ReasoningProcess {
	public:
		ReasoningProcess(const ReasoningTask &task) : task_(task) {};

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

		// TODO: how to realize pause/stop/continue?
		//          - is there some cpp lib for threads that could do it?
		//             but what if reasoner has started sub-threads?
		//          - else need to toggle a flag that IReasoner implementations can
		//             read in "answer" function
		void startReasoning();
		void pauseReasoning();
		void stopReasoning();
		void continueReasoning();

	private:
		ReasoningTask task_;
		ReasoningStatus status_;
	};
}

#endif //__KNOWROB_REASONING_PROCESS_H__
