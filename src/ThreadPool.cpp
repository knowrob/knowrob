/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <stdexcept>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/ThreadPool.h>

using namespace knowrob;

ThreadPool::ThreadPool(uint32_t maxNumThreads)
: maxNumThreads_(maxNumThreads),
  numFinishedThreads_(0)
{
	// NOTE: do not add worker threads in the constructor.
	//  The problem is the virtual initializeWorker function that could be called
	//  in this case before a subclass of ThreadPool overrides it.
	//  i.e. if the thread starts before construction is complete.
	KB_DEBUG("Maximum number of threads: {}.", maxNumThreads);
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
		if(workQueue_.size()>1 || workerThreads_.empty()) {
			// add another thread if max num not reached yet
			if(workerThreads_.size() < (maxNumThreads_ + numFinishedThreads_)) {
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
	if(workQueue_.empty()) {
		return {};
	}
	else {
		std::shared_ptr<ThreadPool::Runner> x = workQueue_.front();
		workQueue_.pop();
		return x;
	}
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
		threadPool_->numFinishedThreads_ += 1;
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

	isTerminated_ = true;
	// tell the thread pool that a worker thread exited
	// FIXME: seems that during pool destruction, any override of finalizeWorker
	//   is not called! it's not really critical because it only concerns system shutdown
	//   as pools are usually used for the whole duration of program execution.
	threadPool_->finalizeWorker();
	// note: counter indicates that there are finished threads in workerThreads_ list.
	threadPool_->numFinishedThreads_ += 1;
	KB_DEBUG("Worker terminated.");
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
	// TODO: provide an interface to notify runner implementation about stop request.
	//    but it wouldn't be of use for PrologReasoner at the moment.
	hasStopRequest_ = true;
	// wait for the runner to be finished if requested
	if(wait) {
		join();
	}
}
