/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_THREAD_POOL_H__
#define __KNOWROB_THREAD_POOL_H__

#include <queue>
#include <mutex>
#include <string>
#include <list>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <thread>

namespace knowrob {
	/**
	 * A pool of worker threads waiting on tasks to be pushed
	 * into a work queue.
	 */
	class ThreadPool {
	public:
		ThreadPool(uint32_t maxNumThreads);
		~ThreadPool();
	
		// forward declarations
		class Worker;
		class Runner;
		
		/** Pushes a goal for a worker.
		 * The goal is assigned to a worker thread when one is available.
		 * @goal the work goal
		 */
		void pushWork(const std::shared_ptr<ThreadPool::Runner> &goal);
		
		/**
		 * @return true if the work queue is currently not empty.
		 */
		bool hasWork() const;
		
		
		/** An object that provides a run function which is evaluated
		 * in a worker thread.
		 */
		class Runner {
		public:
			Runner();
			~Runner();
			
			/** Wait until run function has exited.
			 */
			void join();
			
			/** Run the computation in a worker thread.
			 */
			virtual void run() = 0;
			
			/** Stop the runner.
			 * @wait call blocks until runner exited if true.
			 */
			virtual void stop(bool wait);
			
			/**
			 * @return true if the runner was requested to stop.
			 */
			bool hasStopRequest();
		
		protected:
			std::atomic<bool> isRunning_;
			std::atomic<bool> hasStopRequest_;
			std::mutex mutex_;
			std::condition_variable finishedCV_;
			
			void runInternal();
			
			friend class ThreadPool::Worker;
		};
		
		/** A worker thread that pulls work goals from the work queue of a thread pool.
		 */
		class Worker {
		public:
			Worker(ThreadPool *thread_pool);
			~Worker();
		
		protected:
			ThreadPool *threadPool_;
			std::thread thread_;
			std::mutex threadM_;
			std::shared_ptr<ThreadPool::Runner> goal_;

			std::atomic<bool> isTerminated_;
			std::atomic<bool> hasTerminateRequest_;

			void run();
		};

	private:
		std::list<Worker*> workerThreads_;
		std::queue<std::shared_ptr<ThreadPool::Runner>> workQueue_;
		
		std::condition_variable workCV_;
		mutable std::mutex workMutex_;
		
		uint32_t maxNumThreads_;
		
		std::shared_ptr<ThreadPool::Runner> popWork();
		
		/**
		 */
		virtual bool initializeWorker() { return true; }
		
		/**
		 */
		virtual void finalizeWorker() {}
	};
};

#endif //__KNOWROB_THREAD_POOL_H__

