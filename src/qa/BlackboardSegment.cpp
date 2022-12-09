/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// logging
#include <spdlog/spdlog.h>
// KnowRob
#include <knowrob/qa/BlackboardSegment.h>

using namespace knowrob;


BlackboardSegment::BlackboardSegment(
	const std::shared_ptr<ReasonerManager> &reasonerManager,
	const ReasoningTask &task)
: reasonerManager_(reasonerManager),
  task_(task)
{}

BlackboardSegment::~BlackboardSegment()
{
	stop(true);
}

void BlackboardSegment::start()
{
	// quit currently active processes
	stop(false);
	// create a process
	process_ = std::shared_ptr<BlackboardSegment::Runner>(new BlackboardSegment::Runner(task_));
	// set it as a goal of a worker thread
	// FIXME: it could be problematic if only limited threads are available
	//   to start segments before they have received input. maybe it would be better to react
	//   on first input by pushing the goal instead of doing it here.
	//   else maybe all threads will be blocked by workers waiting on input.
	reasonerManager_->pushGoal(process_);
}

void BlackboardSegment::stop(bool wait)
{
	if(process_.get()) {
		process_->stop(wait);
		process_ = std::shared_ptr<BlackboardSegment::Runner>();
	}
}


BlackboardSegment::Runner::Runner(const ReasoningTask &task)
: IRunner(),
  task_(task)
{
}

void BlackboardSegment::Runner::run()
{
	// generate an identifier for the query request
	uint32_t queryID = reinterpret_cast<std::uintptr_t>(&task_);
	// tell the reasoner that a new request has been made
	task_.reasoner()->startQuery(queryID,
		task_.outputStream(), task_.goal());
	
	// loop until end of input queue has been reached, or stop has been requested
	while(knowrob::isRunning()) {
		// pop front element from queue, block until queue is non empty
		QueryResultPtr input = task_.inputStream()->pop_front();
		// handle stop request
		if(hasStopRequest()) break;
		
		if(QueryResultStream::isEOS(input)) {
			// EOS message
			// note: EOS is not pushed as bindings to the reasoner, but instead
			//   EOS is indicated to the reasoner by calling finishQuery below.
			break;
		}
		else {
			// push bindings into reasoner
			// TODO: it may be better to apply bindings here, and to
			//       also pass an instantiated query to the reasoner?
			task_.reasoner()->pushQueryBindings(queryID, input);
		}
	}
	
	// tell the reasoner to finish up.
	// if hasStopRequest_=true it means that the reasoner is requested
	// to immediately shutdown. however, note that not all reasoner
	// implementations may support this and may take longer to stop anyways.
	task_.reasoner()->finishQuery(queryID, hasStopRequest());
}

void BlackboardSegment::Runner::stop(bool wait)
{
	// toggle stop request flag
	{
		std::lock_guard<std::mutex> lk(mutex_);
		hasStopRequest_ = true;
	}
	// send "EOS" message, this might wake up a blocking call to pop_front() above
	task_.inputStream()->push(QueryResultStream::eos());
	// wait if requested
	IRunner::stop(wait);
}

