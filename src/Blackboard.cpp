/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/Blackboard.h>

#include <memory>

using namespace knowrob;

Blackboard::Blackboard(
	const std::shared_ptr<ReasonerManager> &reasonerManager,
	const std::shared_ptr<QueryResultQueue> &outputQueue,
	const std::shared_ptr<Query> &goal)
: reasonerManager_(reasonerManager),
  outputQueue_(outputQueue),
  goal_(goal)
{
	// decompose the reasoning task
	inputStream_ = std::make_shared<QueryResultBroadcaster>();
	inputChannel_ = QueryResultStream::Channel::create(inputStream_);
	outBroadcaster_ = std::make_shared<QueryResultBroadcaster>();
	
	// create a channel of outputQueue_, and add it as subscriber
	auto queueChannel = QueryResultStream::Channel::create(outputQueue_);
	outBroadcaster_->addSubscriber(queueChannel);

	// decompose into atomic formulas
	decompose(goal->formula(), inputStream_, outBroadcaster_);
}

Blackboard::~Blackboard()
{
	stop();
}

void Blackboard::start()
{
	// push empty message into in stream, and close the channel
	inputChannel_->push(QueryResultStream::bos());
	inputChannel_->close();
}

void Blackboard::stop()
{
	for(auto &segment : segments_) {
		segment->stop();
	}
}

void Blackboard::decompose(const std::shared_ptr<Formula> &phi,
		std::shared_ptr<QueryResultBroadcaster> &in,
		std::shared_ptr<QueryResultBroadcaster> &out)
{
	switch(phi->type()) {
	case FormulaType::PREDICATE: {
		decomposePredicate(
			std::static_pointer_cast<PredicateFormula>(phi),
			in, out);
		break;
	}
	case FormulaType::CONJUNCTION: {
		decomposeConjunction(
			std::static_pointer_cast<ConjunctionFormula>(phi),
			in, out);
		break;
	}
	case FormulaType::DISJUNCTION: {
		decomposeDisjunction(
			std::static_pointer_cast<DisjunctionFormula>(phi),
			in, out);
		break;
	}
	default:
		KB_WARN("Ignoring unknown formula type '{}'.", (int)phi->type());
		break;
	}
}

void Blackboard::decomposePredicate(
		const std::shared_ptr<PredicateFormula> &phi,
		std::shared_ptr<QueryResultBroadcaster> &in,
		std::shared_ptr<QueryResultBroadcaster> &out)
{
	std::shared_ptr<Blackboard::Stream> reasonerIn;
	std::shared_ptr<Blackboard::Segment> segment;
	std::shared_ptr<QueryResultStream::Channel> reasonerInChan, reasonerOutChan;

	// get ensemble of reasoner
	std::list<std::shared_ptr<IReasoner>> ensemble =
		reasonerManager_->getReasonerForPredicate(phi->predicate()->indicator());
	// create a subquery for the predicate p
	std::shared_ptr<Query> subq = std::make_shared<Query>(phi);

    if (ensemble.empty()) {
		// TODO: avoid redundant string formatting by supporting same syntax for QueryErrors
		//throw QueryError("no reasoner registered for query `{}`", *phi);
		KB_WARN("no reasoner registered for query `{}`", *phi);
		throw QueryError("no reasoner available for a predicate");
	}

    for(std::shared_ptr<IReasoner> &r : ensemble) {
		// create IO channels
		reasonerOutChan = QueryResultStream::Channel::create(out);
		reasonerIn = std::make_shared<Blackboard::Stream>(
			r,reasonerOutChan,subq);
		reasonerInChan = QueryResultStream::Channel::create(reasonerIn);
		in->addSubscriber(reasonerInChan);
		// create a new segment
		segments_.push_back(std::make_shared<Segment>(reasonerIn,out));
	}
}

void Blackboard::decomposeConjunction(
		const std::shared_ptr<ConjunctionFormula> &phi,
		std::shared_ptr<QueryResultBroadcaster> &firstQueue,
		std::shared_ptr<QueryResultBroadcaster> &lastQueue)
{
	std::shared_ptr<QueryResultBroadcaster> in = firstQueue;
	std::shared_ptr<QueryResultBroadcaster> out;
	
	// TODO: compute a dependency relation between atomic formulae of a query.
	//   atomic formulae that are independent can be evaluated concurrently.
	
	// add blackboard segments for each predicate in the conjunction
	int counter = 1;
	int numFormulae = phi->formulae().size();
	for(const std::shared_ptr<Formula> &psi : phi->formulae()) {
		if(numFormulae == counter) {
			out = lastQueue;
		}
		else {
			out = std::make_shared<QueryResultBroadcaster>();
		}
		decompose(psi, in, out);
		// output queue of this predicate is used as input for the next one
		in = out;
		counter += 1;
	}
}

void Blackboard::decomposeDisjunction(
		const std::shared_ptr<DisjunctionFormula> &phi,
		std::shared_ptr<QueryResultBroadcaster> &in,
		std::shared_ptr<QueryResultBroadcaster> &out)
{
	// add blackboard segments for each formula in the disjunction.
	// all having the same input and out stream.
	for(const std::shared_ptr<Formula> &psi : phi->formulae()) {
		decompose(psi, in, out);
	}
}


Blackboard::Stream::Stream(
	const std::shared_ptr<IReasoner> &reasoner,
	const std::shared_ptr<QueryResultStream::Channel> &outputStream,
	const std::shared_ptr<Query> &goal)
: QueryResultStream(),
  queryID_(reinterpret_cast<std::uintptr_t>(this)),
  reasoner_(reasoner),
  isQueryOpened_(true),
  hasStopRequest_(false)
{
	// tell the reasoner that a new request has been made
	reasoner_->startQuery(queryID_, outputStream, goal);
}

Blackboard::Stream::~Stream()
{
	stop();
}

void Blackboard::Stream::push(const QueryResultPtr &msg)
{
	if(QueryResultStream::isEOS(msg)) {
		// tell the reasoner to finish up.
		// if hasStopRequest_=true it means that the reasoner is requested
		// to immediately shutdown. however, note that not all reasoner
		// implementations may support this and may take longer to stop anyways.
		if(isQueryOpened()) {
			isQueryOpened_ = false;
			// FIXME: need to call finishQuery if exception happens in runner!!
			//          else consumer will never be called with EOS
			reasoner_->finishQuery(queryID_, hasStopRequest());
		}
	}
	else if(!isQueryOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	}
	else {
		// push substitution to reasoner
		reasoner_->pushSubstitution(queryID_, msg);
	}
}

void Blackboard::Stream::stop()
{
	hasStopRequest_ = true;
	close();
}


Blackboard::Segment::Segment(const std::shared_ptr<Blackboard::Stream> &in,
	const std::shared_ptr<QueryResultBroadcaster> &out)
: in_(in), out_(out)
{}

void Blackboard::Segment::stop()
{
	in_->stop();
}

