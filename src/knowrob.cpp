/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// logging
#include <spdlog/spdlog.h>
// STD
#include <stdexcept>
#include <sstream>
// KnowRob
#include <knowrob/knowrob.h>

using namespace knowrob;

bool isRunning() { return true; }

ThreadPool::ThreadPool(uint32_t numInitialThreads, uint32_t maxNumThreads)
: maxNumThreads_(maxNumThreads)
{
	for(int i=0; i<numInitialThreads; ++i) {
		WorkerThread *thread = new WorkerThread(this);
		availableThreads_.push_back(thread);
		allThreads_.push_back(thread);
	}
}

ThreadPool::~ThreadPool()
{
	for(auto &thread : allThreads_) {
		delete thread;
	}
}

void ThreadPool::pushGoal(const std::shared_ptr<IRunner> &goal)
{
	// TODO: do not block here until a thread is free, add
	//       to a queue in case no thread is available or can be created.
	claim()->setGoal(goal);
}

WorkerThread* ThreadPool::claim()
{
	std::lock_guard<std::mutex> scoped_lock(poolMutex_);
	WorkerThread *thread;
	if(availableThreads_.empty()) {
		// simply create a new engine in case none is available at this time
		// TODO: cap at maxNumThreads and wait in case max would exceed here
		thread = new WorkerThread(this);
		allThreads_.push_back(thread);
	} else {
		thread = availableThreads_.front();
		availableThreads_.pop_front();
	}
	return thread;
}

void ThreadPool::release(WorkerThread *thread)
{
	std::lock_guard<std::mutex> scoped_lock(poolMutex_);
	availableThreads_.push_back(thread);
}


WorkerThread::WorkerThread(ThreadPool *threadPool)
: threadPool_(threadPool),
  isClaimed_(false),
  isTerminated_(false),
  hasTerminateRequest_(false),
  thread_(&WorkerThread::run, this)
{
}

WorkerThread::~WorkerThread()
{
	{
		std::unique_lock<std::mutex> lk(threadM_);
		hasTerminateRequest_ = true;
	}
	threadCV_.notify_one();
	thread_.join();
}

void WorkerThread::setGoal(const std::shared_ptr<IRunner> &goal)
{
	{
		std::lock_guard<std::mutex> lk(threadM_);
		if(isClaimed_) {
			throw std::runtime_error("the thread is claimed by someone else.");
		}
		if(isTerminated_) {
			throw std::runtime_error("the thread is terminated.");
		}
		goal_ = goal;
		isClaimed_ = true;
	}
	// notify the thread that a claim has been made
	threadCV_.notify_one();
}

void WorkerThread::cancelGoal(bool wait)
{
	if(goal_.get() != NULL) {
		goal_->stop(wait);
	}
}

void WorkerThread::run()
{
	std::string threadID; {
		std::stringstream ss;
		ss << thread_.get_id();
		threadID = ss.str();
	}
	spdlog::debug("WorkerThread {} started.", threadID.c_str());
	
	if(threadPool_->initializeWorker(this)) {
		spdlog::debug("WorkerThread {} initialization failed.", threadID.c_str());
		return;
	}
	
	// loop until the application exits
	while(knowrob::isRunning()) {
		// wait for a claim
		{
			std::unique_lock<std::mutex> lk(threadM_);
			threadCV_.wait(lk, [this]{
				return hasTerminateRequest_ || isClaimed_;
			});
			if(hasTerminateRequest_) break;
		}
		spdlog::debug("WorkerThread {} has a new goal.", threadID.c_str());
		
		// do the work
		try {
			goal_->runInternal();
		}
		catch(const std::exception& e) {
			spdlog::warn("WorkerThread {} runner error: {}.", threadID.c_str(), e.what());
		}
		
		// add the worker thread to the thread pool again
		threadPool_->release(this);
		spdlog::debug("WorkerThread {} finished a goal.", threadID.c_str());
	}
	
	// TODO: need to stop currently active workers?
	threadPool_->finalizeWorker(this);

	spdlog::debug("WorkerThread {} terminated.", threadID.c_str());
	isTerminated_ = true;
}


IRunner::IRunner()
: isRunning_(false),
  hasStopRequest_(false)
{}

IRunner::~IRunner()
{
	stop(true);
}


bool IRunner::hasStopRequest()
{
	std::lock_guard<std::mutex> lk(mutex_);
	return hasStopRequest_;
}

void IRunner::join()
{
	{
		std::lock_guard<std::mutex> lk(mutex_);
		if(!isRunning_) return;
	}
	std::unique_lock<std::mutex> lk(mutex_);
	finishedCV_.wait(lk, [this]{
		return !isRunning_;
	});
}

void IRunner::runInternal()
{
	// toggle isRunning flag on
	{
		std::lock_guard<std::mutex> lk(mutex_);
		isRunning_ = true;
	}
	// do the work
	run();
	// toggle isRunning flag off, and notify everybody waiting in IRunner::stop()
	{
		std::lock_guard<std::mutex> lk(mutex_);
		isRunning_ = false;
	}
	finishedCV_.notify_all();
}

void IRunner::stop(bool wait)
{
	// toggle stop request flag on
	{
		std::lock_guard<std::mutex> lk(mutex_);
		hasStopRequest_ = true;
		if(!isRunning_) return;
	}
	// wait for the runner to be finished if requested
	if(wait) {
		join();
	}
}


