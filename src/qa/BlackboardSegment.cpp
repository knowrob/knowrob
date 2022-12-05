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
	const std::shared_ptr<IReasoner> &reasoner,
	const std::shared_ptr<QueryResultQueue> &inputQueue,
	const std::shared_ptr<QueryResultStream> &outputQueue,
	const std::shared_ptr<Query> &goal)
: reasonerManager_(reasonerManager),
  reasoner_(reasoner),
  inputQueue_(inputQueue),
  outputQueue_(outputQueue),
  goal_(goal)
{}

BlackboardSegment::~BlackboardSegment()
{
	stopReasoningProcess(true);
}

void BlackboardSegment::startReasoningProcess()
{
	process_ = reasonerManager_->submit(
		ReasoningTask(reasoner_, inputQueue_, outputQueue_, goal_));
}

void BlackboardSegment::stopReasoningProcesses(bool wait)
{
	if(process_.get() != NULL) {
		process_->stop(wait);
		process_ = std::shared_ptr<ReasoningProcess>();
	}
}

