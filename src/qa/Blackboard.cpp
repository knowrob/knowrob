/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

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
	out->addSubscriber(outputQueue_);
	decompose(goal->formula(), in, out);
	
	// TODO: push empty and EOS message to input stream
	
	// start the reasoning
	for(std::shared_ptr<BlackboardSegment> &segment : segments_) {
		segment->startReasoningProcess();
	}
}

Blackboard::~Blackboard()
{
	stopReasoningProcesses();
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
		reasonerIn = std::shared_ptr<QueryResultQueue>(new QueryResultQueue);
		reasonerOut = std::shared_ptr<QueryResultBroadcast>(new QueryResultBroadcast);
		in->addSubscriber(reasonerIn.get());
		reasonerOut->addSubscriber(out.get());
		// create the blackboard segment
		segments_.push_back(std::shared_ptr<BlackboardSegment>(
			new BlackboardSegment(r,reasonerIn,reasonerOut,subq)));
	}
}

void Blackboard::decomposeConjunction(
		const std::shared_ptr<ConjunctionFormula> &phi,
		std::shared_ptr<QueryResultBroadcast> &firstQueue,
		std::shared_ptr<QueryResultBroadcast> &lastQueue)
{
	std::shared_ptr<QueryResultBroadcast> in = firstQueue;
	std::shared_ptr<QueryResultBroadcast> out;
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

void Blackboard::stopReasoningProcesses()
{
	for(std::shared_ptr<BlackboardSegment>& segment : segments_) {
		segment->stopReasoningProcess();
	}
}

