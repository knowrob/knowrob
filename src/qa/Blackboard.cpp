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
	std::shared_ptr<QueryResultQueue> inputQueue =
		std::shared_ptr<QueryResultQueue>(new QueryResultQueue);
	decompose(goal->formula(), inputQueue, outputQueue_);
	// TODO: need to push one message to inputQueue to trigger the reasoning processes!
	
	// start the reasoning
	for(std::shared_ptr<BlackboardSegment> &segment : segments_) {
		segment->startReasoningProcesses();
	}
}

Blackboard::~Blackboard()
{
	stopReasoningProcesses();
}

void Blackboard::decompose(const std::shared_ptr<Formula> &phi,
		std::shared_ptr<QueryResultQueue> &in,
		std::shared_ptr<QueryResultQueue> &out)
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
		// TODO write warning about unknown formula type
		break;
	}
}

void Blackboard::decomposePredicate(
		const std::shared_ptr<PredicateFormula> &phi,
		std::shared_ptr<QueryResultQueue> &in,
		std::shared_ptr<QueryResultQueue> &out)
{
	std::list<std::shared_ptr<IReasoner>> essemble =
		reasonerManager_->getEssembleForPredicate(phi->predicate()->indicator());
	// create a subquery for the predicate p
	std::shared_ptr<Query> subq = std::shared_ptr<Query>(new Query(phi));

	for(std::shared_ptr<IReasoner> &r : essemble) {
		std::shared_ptr<BlackboardSegment> segment =
			std::shared_ptr<BlackboardSegment>(new BlackboardSegment(reasonerManager_,in,out,subq));
		segment->addReasoner(r);
		segments_.push_back(segment);
	}
}

void Blackboard::decomposeConjunction(
		const std::shared_ptr<ConjunctionFormula> &phi,
		std::shared_ptr<QueryResultQueue> &firstQueue,
		std::shared_ptr<QueryResultQueue> &lastQueue)
{
	std::shared_ptr<QueryResultQueue> in = firstQueue;
	std::shared_ptr<QueryResultQueue> out;
	// add blackboard segments for each predicate in the conjunction
	int counter = 1;
	int numFormulae = phi->formulae().size();
	for(const std::shared_ptr<Formula> &psi : phi->formulae()) {
		if(numFormulae == counter) {
			out = lastQueue;
		}
		else {
			out = std::shared_ptr<QueryResultQueue>(new QueryResultQueue);
		}
		decompose(psi, in, out);
		// output queue of this predicate is used as input for the next one
		in = out;
		counter += 1;
	}
	// TODO: merge segments if possible, i.e. if a reasoner supports evaluation of conjunctive goals
}

void Blackboard::decomposeDisjunction(
		const std::shared_ptr<DisjunctionFormula> &phi,
		std::shared_ptr<QueryResultQueue> &in,
		std::shared_ptr<QueryResultQueue> &out)
{
	// add blackboard segments for each formula in the disjunction.
	// all having the same input and out queue.
	for(const std::shared_ptr<Formula> &psi : phi->formulae()) {
		decompose(psi, in, out);
	}
	// TODO: merge segments if possible, i.e. if a reasoner supports evaluation of disjunctive goals
}

void Blackboard::stopReasoningProcesses()
{
	for(std::shared_ptr<BlackboardSegment>& segment : segments_) {
		segment->stopReasoningProcesses();
	}
}

