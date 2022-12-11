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
#include <knowrob/ThreadPool.h>

using namespace knowrob;

ThreadPool::ThreadPool(uint32_t maxNumThreads)
: maxNumThreads_(maxNumThreads)
{
	// create one worker thread
	workerThreads_.push_back(new Worker(this));
}

ThreadPool::~ThreadPool()
{
	for(Worker *t : workerThreads_) {
		delete t;
	}
	workerThreads_.clear();
}

void ThreadPool::pushWork(const std::shared_ptr<ThreadPool::Runner> &goal)
{
	{
		std::lock_guard<std::mutex> scoped_lock(workMutex_);
		workQueue_.push(goal);
		if(workQueue_.size()>1) {
			// add another thread if max num not reached yet
			if(workerThreads_.size() < maxNumThreads_) {
				workerThreads_.push_back(new Worker(this));
			}
		}
	}
	// wake up a worker if any is sleeping
	workCV_.notify_one();
}

std::shared_ptr<ThreadPool::Runner> ThreadPool::popWork()
{
	std::lock_guard<std::mutex> scoped_lock(workMutex_);
	std::shared_ptr<ThreadPool::Runner> x = workQueue_.front();
	workQueue_.pop();
	return x;
}

bool ThreadPool::hasWork() const
{
	std::lock_guard<std::mutex> scoped_lock(workMutex_);
	return workQueue_.empty();
}


ThreadPool::Worker::Worker(ThreadPool *threadPool)
: threadPool_(threadPool),
  isTerminated_(false),
  hasTerminateRequest_(false),
  thread_(&Worker::run, this)
{
}

ThreadPool::Worker::~Worker()
{
	hasTerminateRequest_ = true;
	thread_.join();
}

void ThreadPool::Worker::run()
{
	std::string threadID; {
		std::stringstream ss;
		ss << thread_.get_id();
		threadID = ss.str();
	}
	spdlog::debug("WorkerThread {} started.", threadID.c_str());
	
	// let the pool do some thread specific initialization
	if(threadPool_->initializeWorker()) {
		spdlog::debug("WorkerThread {} initialization failed.", threadID.c_str());
		return;
	}
	
	// loop until the application exits
	while(!hasTerminateRequest_) {
		// wait for a claim
		if(!threadPool_->hasWork()) {
			std::unique_lock<std::mutex> lk(threadM_);
			threadPool_->workCV_.wait(lk, [this]{
				return hasTerminateRequest_ || threadPool_->hasWork();
			});
			if(hasTerminateRequest_) break;
		}
		
		// pop work from queue
		goal_ = threadPool_->popWork();
		spdlog::debug("WorkerThread {} has a new goal.", threadID.c_str());
		
		// do the work
		try {
			if(goal_.get()!=NULL) {
				goal_->runInternal();
			}
		}
		catch(const std::exception& e) {
			spdlog::warn("WorkerThread {} runner error: {}.", threadID.c_str(), e.what());
		}
		
		// add the worker thread to the thread pool again
		spdlog::debug("WorkerThread {} finished a goal.", threadID.c_str());
	}
	
	// tell the thread pool that a worker thread exited
	threadPool_->finalizeWorker();

	spdlog::debug("WorkerThread {} terminated.", threadID.c_str());
	isTerminated_ = true;
}


ThreadPool::Runner::Runner()
: isRunning_(false),
  hasStopRequest_(false)
{}

ThreadPool::Runner::~Runner()
{
	stop(true);
}


bool ThreadPool::Runner::hasStopRequest()
{
	return hasStopRequest_;
}

void ThreadPool::Runner::join()
{
	if(!isRunning_) return;
	std::unique_lock<std::mutex> lk(mutex_);
	finishedCV_.wait(lk, [this]{ return !isRunning_; });
}

void ThreadPool::Runner::runInternal()
{
	// toggle isRunning flag on
	isRunning_ = true;
	// do the work
	run();
	// toggle isRunning flag off, and notify everybody waiting in Runner::stop()
	isRunning_ = false;
	finishedCV_.notify_all();
}

void ThreadPool::Runner::stop(bool wait)
{
	// toggle stop request flag on
	hasStopRequest_ = true;
	if(!isRunning_) return;
	// wait for the runner to be finished if requested
	if(wait) {
		join();
	}
}


