/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_H__
#define __KNOWROB_H__

#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <thread>

namespace knowrob {
	bool isRunning();
	
	/**
	 */
	class ParserError : public std::runtime_error {
	public:
		/**
		 */
		ParserError(const std::string& what = "") : std::runtime_error(what) {}
	};
	
	// forward declaration
	class ThreadPool;
	
	class IRunner {
	public:
		virtual void run() = 0;
	};

	/**
	 */
	class WorkerThread {
	public:
		WorkerThread(ThreadPool *thread_pool);
		~WorkerThread();
		
		void setGoal(const std::shared_ptr<IRunner> &goal);
	
	protected:
		ThreadPool *threadPool_;
		std::thread thread_;
		std::mutex threadM_;
		std::condition_variable threadCV_;
		std::shared_ptr<IRunner> goal_;

		bool isClaimed_;
		bool isTerminated_;
		bool hasTerminateRequest_;

		void run();
	};


	/**
	 * A simple thread pool implementation for Prolog engines.
	 * The pool starts at a user-defined size, but may grow in
	 * case many parallel requests are issued.
	 *
	 * @author Daniel Beßler
	 */
	class ThreadPool {
	public:
		ThreadPool(uint32_t numInitialThreads, uint32_t maxNumThreads=0);
		~ThreadPool();

		/**
		 * Claim a Prolog engine. This claim is exclusive.
		 * To allow others using the engine again, the
		 * claim needs to be lifted by calling *release*.
		 */
		WorkerThread* claim();

 		/**
		 * Release the claim for an engine thread.
		 */
		void release(WorkerThread *thread);

	private:
		std::list<WorkerThread*> availableThreads_;
		std::list<WorkerThread*> allThreads_;
		std::mutex poolMutex_;
		uint32_t maxNumThreads_;
	};
};

#endif //__KNOWROB_H__

