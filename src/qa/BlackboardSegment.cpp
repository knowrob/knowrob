/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/qa/BlackboardSegment.h>

using namespace knowrob;

BlackboardSegment::BlackboardSegment(
	const std::shared_ptr<ReasonerManager> &reasonerManager,
	const std::shared_ptr<QueryResultQueue> &inputQueue,
	const std::shared_ptr<QueryResultQueue> &outputQueue,
	const std::shared_ptr<Query> &goal)
: reasonerManager_(reasonerManager),
  inputQueue_(inputQueue),
  outputQueue_(outputQueue),
  goal_(goal),
  isRunning_(false)
{}

BlackboardSegment::~BlackboardSegment()
{
	stopReasoningProcesses();
}

void BlackboardSegment::addReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	{
		std::lock_guard<std::mutex> lk(mutex_);
		reasoner_.push_back(reasoner);
	}
	if(isRunning_) {
		startReasoner(reasoner);
	}
}

void BlackboardSegment::removeReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	if(isRunning_) {
		stopReasoner(reasoner);
	}
	{
		std::lock_guard<std::mutex> lk(mutex_);
		reasoner_.remove(reasoner);
	}
}

void BlackboardSegment::startReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	processes_[reasoner] = reasonerManager_->submitTask(
		ReasoningTask(reasoner, inputQueue_, outputQueue_, goal_));
}

void BlackboardSegment::stopReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	auto it = processes_.find(reasoner);
	if(it != processes_.end()) {
		reasonerManager_->cancelTask(it->second);
		processes_.erase(it);
	}
}

void BlackboardSegment::startReasoningProcesses()
{
	std::lock_guard<std::mutex> lk(mutex_);
	if(!isRunning_) {
		for(const std::shared_ptr<IReasoner>& x : reasoner_) {
			startReasoner(x);
		}
		isRunning_ = true;
	}
}

void BlackboardSegment::stopReasoningProcesses()
{
	std::lock_guard<std::mutex> lk(mutex_);
	if(isRunning_) {
		for(const std::shared_ptr<IReasoner>& x : reasoner_) {
			stopReasoner(x);
		}
		isRunning_ = false;
	}
}

