/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/reasoning/ReasonerManager.h>

using namespace knowrob;

ReasonerManager::ReasonerManager(uint32_t numInitialThreads, uint32_t maxNumThreads)
: threadPool_(numInitialThreads, maxNumThreads)
{}

void ReasonerManager::addReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	reasonerPool_.push_back(reasoner);
}

void ReasonerManager::removeReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	reasonerPool_.remove(reasoner);
}

std::list<std::shared_ptr<IReasoner>> ReasonerManager::getReasonerForPredicate(const PredicateIndicator &predicate)
{
	std::list<std::shared_ptr<IReasoner>> out;
	for(auto &x : reasonerPool_) {
		if(x->canReasonAbout(predicate)) {
			out.push_back(x);
		}
	}
	return out;
}

std::shared_ptr<ReasoningProcess> ReasonerManager::submit(const ReasoningTask &tsk)
{
	std::shared_ptr<ReasoningProcess> proc =
		std::shared_ptr<ReasoningProcess>(new ReasoningProcess(tsk));
	// claim a thread, and set the goal for this thread
	threadPool_.claim()->setGoal(proc);
	return proc;
}

