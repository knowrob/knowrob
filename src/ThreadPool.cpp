/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <stdexcept>
#include <sstream>
// KnowRob
#include <knowrob/logging.h>
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
		t->hasTerminateRequest_ = true;
	}
	workCV_.notify_all();
	for(Worker *t : workerThreads_) {
		// note: worker destructor joins the thread
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
	return !workQueue_.empty();
}


ThreadPool::Worker::Worker(ThreadPool *threadPool)
: hasTerminateRequest_(false),
  isTerminated_(false),
  threadPool_(threadPool),
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
	KB_DEBUG("Worker started.");
	// let the pool do some thread specific initialization
	if(!threadPool_->initializeWorker()) {
		KB_ERROR("Worker initialization failed.");
		isTerminated_ = true;
		// FIXME: remove from workerThreads_
		return;
	}
	KB_DEBUG("Worker initialized.");
	hasTerminateRequest_ = false;
	
	// loop until the application exits
	while(!hasTerminateRequest_) {
		// wait for a claim
		{
			KB_DEBUG("Worker going to sleep.");
			std::unique_lock<std::mutex> lk(threadPool_->workMutex_);
			threadPool_->workCV_.wait(lk, [this]{
				return hasTerminateRequest_ || !threadPool_->workQueue_.empty();
			});
			KB_DEBUG("Worker woke up.");
		}
		if(hasTerminateRequest_) {
			KB_DEBUG("Worker has terminate request.");
			break;
		}
		
		// pop work from queue
		goal_ = threadPool_->popWork();
		// do the work
		if(goal_) {
			KB_DEBUG("Worker has a new goal.");
			goal_->runInternal();
			KB_DEBUG("Work finished.");
		}
	}
	
	// tell the thread pool that a worker thread exited
	// FIXME: remove from workerThreads_
	threadPool_->finalizeWorker();

	KB_DEBUG("Worker terminated.");
	isTerminated_ = true;
}


ThreadPool::Runner::Runner()
: isTerminated_(false),
  hasStopRequest_(false)
{}

ThreadPool::Runner::~Runner()
{
	stop(true);
}

void ThreadPool::Runner::join()
{
	if(!isTerminated()) {
		std::unique_lock<std::mutex> lk(mutex_);
		finishedCV_.wait(lk, [this]{ return isTerminated(); });
	}
}

void ThreadPool::Runner::runInternal()
{
	// do the work
	try {
		run();
	}
	catch(const std::exception& e) {
		KB_WARN("Worker error: {}.", e.what());
	}
	// toggle flag
	isTerminated_ = true;
	finishedCV_.notify_all();
}

void ThreadPool::Runner::stop(bool wait)
{
	// toggle stop request flag on
	hasStopRequest_ = true;
	if(isTerminated()) return;
	// wait for the runner to be finished if requested
	if(wait) {
		join();
	}
}
