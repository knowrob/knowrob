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
#include <knowrob/qa/Blackboard.h>

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
	std::shared_ptr<QueryResultBroadcast> in =
		std::shared_ptr<QueryResultBroadcast>(new QueryResultBroadcast);
	std::shared_ptr<QueryResultBroadcast> out =
		std::shared_ptr<QueryResultBroadcast>(new QueryResultBroadcast);
	out->addSubscriber(outputQueue_.get());
	decompose(goal->formula(), in, out);
	
	// push BOS (empty substitution) followed by EOS message to input stream
	in->push(QueryResultStream::bos());
	in->push(QueryResultStream::eos());
	
	// start the reasoning
	for(std::shared_ptr<BlackboardSegment> &segment : segments_) {
		segment->start();
	}
}

Blackboard::~Blackboard()
{
	stop();
}

void Blackboard::decompose(const std::shared_ptr<Formula> &phi,
		std::shared_ptr<QueryResultBroadcast> &in,
		std::shared_ptr<QueryResultBroadcast> &out)
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
		spdlog::warn("Ignoring unknown formula type '{}' in query.", phi->type());
		break;
	}
}

void Blackboard::decomposePredicate(
		const std::shared_ptr<PredicateFormula> &phi,
		std::shared_ptr<QueryResultBroadcast> &in,
		std::shared_ptr<QueryResultBroadcast> &out)
{
	std::shared_ptr<QueryResultQueue> reasonerIn;
	std::shared_ptr<QueryResultBroadcast> reasonerOut;
	// get ensemble of reasoner
	std::list<std::shared_ptr<IReasoner>> ensemble =
		reasonerManager_->getReasonerForPredicate(phi->predicate()->indicator());
	// create a subquery for the predicate p
	std::shared_ptr<Query> subq = std::shared_ptr<Query>(new Query(phi));

	for(std::shared_ptr<IReasoner> &r : ensemble) {
		// create IO streams
		// TODO: reasonerIn could be limited to substitutions of variables that actually
		// appear in subq! this would relax data load into the reasoner, and
		// it would allow to avoid redundant calls, by keeping a cache of
		// already evaluated instantiations. It would also require an additional step
		// to combine output of reasoner with input that was not passed to it via the input stream.
		reasonerIn = std::shared_ptr<QueryResultQueue>(new QueryResultQueue);
		reasonerOut = std::shared_ptr<QueryResultBroadcast>(new QueryResultBroadcast);
		// manage subscriptions needed for the message broadcasting
		in->addSubscriber(reasonerIn.get());
		reasonerOut->addSubscriber(out.get());
		// create the blackboard segment
		segments_.push_back(std::shared_ptr<BlackboardSegment>(new BlackboardSegment(
			reasonerManager_,
			ReasoningTask(r,reasonerIn,reasonerOut,subq)
		)));
	}
}

void Blackboard::decomposeConjunction(
		const std::shared_ptr<ConjunctionFormula> &phi,
		std::shared_ptr<QueryResultBroadcast> &firstQueue,
		std::shared_ptr<QueryResultBroadcast> &lastQueue)
{
	std::shared_ptr<QueryResultBroadcast> in = firstQueue;
	std::shared_ptr<QueryResultBroadcast> out;
	
	// TODO: compute a dependency relation between atomic formulae of a query.
	// atomic formulae that are independent can be evaluated in concurrently.
	
	// add blackboard segments for each predicate in the conjunction
	int counter = 1;
	int numFormulae = phi->formulae().size();
	for(const std::shared_ptr<Formula> &psi : phi->formulae()) {
		if(numFormulae == counter) {
			out = lastQueue;
		}
		else {
			out = std::shared_ptr<QueryResultBroadcast>(new QueryResultBroadcast);
		}
		decompose(psi, in, out);
		// output queue of this predicate is used as input for the next one
		in = out;
		counter += 1;
	}
}

void Blackboard::decomposeDisjunction(
		const std::shared_ptr<DisjunctionFormula> &phi,
		std::shared_ptr<QueryResultBroadcast> &in,
		std::shared_ptr<QueryResultBroadcast> &out)
{
	// add blackboard segments for each formula in the disjunction.
	// all having the same input and out queue.
	for(const std::shared_ptr<Formula> &psi : phi->formulae()) {
		decompose(psi, in, out);
	}
}

void Blackboard::stop()
{
	for(std::shared_ptr<BlackboardSegment>& segment : segments_) {
		segment->stop();
	}
}

