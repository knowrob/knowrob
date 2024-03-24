/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <stdexcept>
#include <utility>
#include <knowrob/Logger.h>
#include <knowrob/ThreadPool.h>
#include <boost/python/errors.hpp>
#include "knowrob/py/PythonError.h"

using namespace knowrob;

namespace knowrob {
	std::shared_ptr<ThreadPool> DefaultThreadPool() {
		static auto pool =
				std::make_shared<ThreadPool>(std::thread::hardware_concurrency());
		return pool;
	}
}

ThreadPool::ThreadPool(uint32_t maxNumThreads)
		: maxNumThreads_(maxNumThreads),
		  numFinishedThreads_(0),
		  numActiveWorker_(0) {
	// NOTE: do not add worker threads in the constructor.
	//  The problem is the virtual initializeWorker function that could be called
	//  in this case before a subclass of ThreadPool overrides it.
	//  i.e. if the thread starts before construction is complete.
	KB_DEBUG("Maximum number of threads: {}.", maxNumThreads);
}

ThreadPool::~ThreadPool() {
	shutdown();
}

void ThreadPool::shutdown() {
	for (Worker *t: workerThreads_) {
		t->hasTerminateRequest_ = true;
	}
	workCV_.notify_all();
	for (Worker *t: workerThreads_) {
		// note: worker destructor joins the thread
		delete t;
	}
	workerThreads_.clear();
}

void ThreadPool::pushWork(const std::shared_ptr<ThreadPool::Runner> &goal, ExceptionHandler exceptionHandler) {
	{
		std::lock_guard<std::mutex> scoped_lock(workMutex_);
		goal->setExceptionHandler(std::move(exceptionHandler));
		workQueue_.push(goal);

		uint32_t numAliveThreads = workerThreads_.size() - numFinishedThreads_;
		uint32_t numAvailableThreads = numAliveThreads - numActiveWorker_;
		if (numAvailableThreads == 0) {
			// add another thread if max num not reached yet
			if (workerThreads_.size() < (maxNumThreads_ + numFinishedThreads_)) {
				workerThreads_.push_back(new Worker(this));
			}
		}
	}
	// wake up a worker if any is sleeping
	workCV_.notify_one();
}

std::shared_ptr<ThreadPool::Runner> ThreadPool::popWork() {
	std::lock_guard<std::mutex> scoped_lock(workMutex_);
	if (workQueue_.empty()) {
		return {};
	} else {
		std::shared_ptr<ThreadPool::Runner> x = workQueue_.front();
		workQueue_.pop();
		return x;
	}
}


ThreadPool::Worker::Worker(ThreadPool *threadPool)
		: hasTerminateRequest_(false),
		  isTerminated_(false),
		  threadPool_(threadPool),
		  thread_(&Worker::run, this) {
}

ThreadPool::Worker::~Worker() {
	hasTerminateRequest_ = true;
	thread_.join();
}

void ThreadPool::Worker::run() {
	KB_DEBUG("Worker started.");
	// let the pool do some thread specific initialization
	if (!threadPool_->initializeWorker()) {
		KB_ERROR("Worker initialization failed.");
		isTerminated_ = true;
		threadPool_->numFinishedThreads_ += 1;
		return;
	}
	KB_DEBUG("Worker initialized.");
	hasTerminateRequest_ = false;
	threadPool_->numActiveWorker_ += 1;

	// loop until the application exits
	while (!hasTerminateRequest_) {
		// wait for a claim
		{
			KB_DEBUG("Worker going to sleep.");
			std::unique_lock<std::mutex> lk(threadPool_->workMutex_);
			threadPool_->numActiveWorker_ -= 1;
			threadPool_->workCV_.wait(lk, [this] {
				return hasTerminateRequest_ || !threadPool_->workQueue_.empty();
			});
			threadPool_->numActiveWorker_ += 1;
			KB_DEBUG("Worker woke up.");
		}
		if (hasTerminateRequest_) {
			KB_DEBUG("Worker has terminate request.");
			break;
		}

		// pop work from queue
		auto goal = threadPool_->popWork();
		// do the work
		if (goal) {
			KB_DEBUG("Worker has a new goal.");
			goal->runInternal();
			KB_DEBUG("Work finished.");
		}
	}

	isTerminated_ = true;
	// tell the thread pool that a worker thread exited
	if (threadPool_->finalizeWorker_) {
		threadPool_->finalizeWorker_();
	}
	// note: counter indicates that there are finished threads in workerThreads_ list.
	threadPool_->numFinishedThreads_ += 1;
	threadPool_->numActiveWorker_ -= 1;
	KB_DEBUG("Worker terminated.");
}


ThreadPool::Runner::Runner()
		: isTerminated_(false),
		  hasStopRequest_(false),
		  isRunning_(false),
		  exceptionHandler_(nullptr) {}

ThreadPool::Runner::~Runner() {
	if(isRunning_) stop(true);
}

void ThreadPool::Runner::join() {
	if (!isTerminated()) {
		std::unique_lock<std::mutex> lk(mutex_);
		finishedCV_.wait(lk, [this] { return isTerminated(); });
	}
}

void ThreadPool::Runner::runInternal() {
	isRunning_ = true;
	isTerminated_ = false;
	hasStopRequest_ = false;
	// do the work
	try {
		run();
	}
	catch (const boost::python::error_already_set &) {
		PythonError py_error;
		if (exceptionHandler_) {
			exceptionHandler_(py_error);
		} else {
			KB_WARN("Worker error in Python code: {}.", py_error.what());
		}
	}
	catch (const std::exception &e) {
		if (exceptionHandler_) {
			exceptionHandler_(e);
		} else {
			KB_WARN("Worker error: {}.", e.what());
		}
	}
	catch (...) {
		KB_WARN("Unknown worker error.");
	}
	// toggle flag
	isTerminated_ = true;
	isRunning_ = false;
	finishedCV_.notify_all();
}

void ThreadPool::Runner::stop(bool wait) {
	// toggle stop request flag on
	hasStopRequest_ = true;
	// wait for the runner to be finished if requested
	if (wait) {
		join();
	}
}
