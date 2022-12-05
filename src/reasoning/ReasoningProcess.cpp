/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/reasoning/ReasoningProcess.h>

using namespace knowrob;

ReasoningProcess::ReasoningProcess(const ReasoningTask &task)
: task_(task),
  isRunning_(false),
  hasStopRequest_(false)
{
}

ReasoningProcess::~ReasoningProcess()
{
	stop(true);
}

void ReasoningProcess::run()
{
	// toggle isRunning flag on
	{
		std::lock_guard<std::mutex> lk(mutex_);
		isRunning_ = true;
	}
	// generate an identifier for the query request
	uint32_t queryID = reinterpret_cast<std::uintptr_t>(&task_);
	// tell the reasoner that a new request has been made
	task_.reasoner()->startQuery(queryID,
		task_.goal(), task_.outputStream());
	
	// loop until end of input queue has been reached, or stop has been requested
	while(knowrob::ok() && !hasStopRequest_) {
		// pop front element from queue, block until queue is non empty
		QueryResultPtr input = task_.inputQueue().front();
		task_.inputQueue().pop();
		
		if(input->hasSolution()) {
			// push bindings into reasoner
			task_.reasoner()->pushQueryBindings(queryID, input);
		}
		else {
			// EOS message
			// note: EOS is not pushed as bindings to the reasoner, but instead
			//   EOS is indicated to the reasoner by calling finishQuery below.
			break;
		}
	}
	
	// tell the reasoner to finish up.
	// if hasStopRequest_=true it means that the reasoner is requested
	// to immediately shutdown. however, note that not all reasoner
	// implementations may support this and may take longer to stop anyways.
	task_.reasoner()->finishQuery(queryID, hasStopRequest_);
	
	// toggle isRunning flag off, and notify everybody waiting in stop()
	{
		std::lock_guard<std::mutex> lk(mutex_);
		isRunning_ = false;
	}
	finishedCV_.notify_all();
}

void ReasoningProcess::stop(bool wait)
{
	// toggle flag for faster shutdown of reasoner
	hasStopRequest_ = true;
	// send "EOS" message, this might wake up a blocking call to front() above
	task_.inputQueue().push(QueryResult::NO_SOLUTION);
	// wait for the process to stop if requested
	if(wait) {
		std::unique_lock<std::mutex> lk(mutex_);
		finishedCV_.wait(lk, [this]{
			return !isRunning_;
		});
	}
}


