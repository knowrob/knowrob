/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_THREAD_POOL_H_
#define KNOWROB_PROLOG_THREAD_POOL_H_

#include "knowrob/ThreadPool.h"

namespace knowrob {
	/**
	 * A pool of threads with attached Prolog engines.
	 * Prolog threads have their own stacks and only share the Prolog heap:
	 * predicates, records, flags and other global non-backtrackable data.
	 */
	class PrologThreadPool : public ThreadPool {
	public:
		/**
		 * @maxNumThreads maximum number of worker threads.
		 */
		explicit PrologThreadPool(uint32_t maxNumThreads=0);

	protected:
		// Override ThreadPool
		bool initializeWorker() override;
		
		// Override ThreadPool
		void finalizeWorker() override;
	};
}

#endif //KNOWROB_PROLOG_THREAD_POOL_H_
