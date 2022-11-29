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
	const boost::shared_ptr<ReasonerManager> &reasonerManager,
	const boost::shared_ptr<QueryResultQueue> &outputQueue,
	const boost::shared_ptr<Query> &goal)
: reasonerManager_(reasonerManager),
  outputQueue_(outputQueue),
  goal_(goal)
{
	// decompose the reasoning task
	boost::shared_ptr<QueryResultQueue> inputQueue =
		boost::shared_ptr<QueryResultQueue>(new QueryResultQueue);
	decompose(goal.formula(), inputQueue, outputQueue_);
	// TODO: need to push one message to inputQueue to trigger the reasoning processes!
	
	// start the reasoning
	for(auto &segment : segments_) {
		segment->startReasoningProcesses();
	}
}

Blackboard::~Blackboard()
{
	stopReasoningProcesses();
}

void Blackboard::decompose(const Formula &phi,
		boost::shared_ptr<QueryResultQueue> &in,
		boost::shared_ptr<QueryResultQueue> &out)
{
	switch(phi.type()) {
	case FormulaType::PREDICATE:
		PredicateFormula *x = (PredicateFormula*)(&phi);
		decomposePredicate(*x);
		break;
	case FormulaType::CONJUNCTION:
		ConjunctionFormula *x = (ConjunctionFormula*)(&phi);
		decomposeConjunction(*x);
		break;
	case FormulaType::DISJUNCTION:
		DisjunctionFormula *x = (DisjunctionFormula*)(&phi);
		decomposeDisjunction(*x, in, out);
		break;
	default:
		// TODO write warning about unknown formula type
		break;
	}
}

void Blackboard::decomposePredicate(
		const PredicateFormula &phi,
		boost::shared_ptr<QueryResultQueue> &in,
		boost::shared_ptr<QueryResultQueue> &out)
{
	std::list<boost::shared_ptr<IReasoner>> essemble =
		reasonerManager_.getEssembleForPredicate(phi.predicate().indicator());
	// create a subquery for the predicate p
	boost::shared_ptr<Query> subq = boost::shared_ptr<Query>(new Query(phi.predicate()));

	for(boost::shared_ptr<IReasoner> &r : essemble) {
		boost::shared_ptr<BlackboardSegment> segment =
			boost::shared_ptr<BlackboardSegment>(new BlackboardSegment(reasonerManager_,in,out,subq));
		segment->addReasoner(r);
		segments_.push_back(segment);
	}
}

void Blackboard::decomposeConjunction(
		const ConjunctionFormula &phi,
		boost::shared_ptr<QueryResultQueue> &firstQueue,
		boost::shared_ptr<QueryResultQueue> &lastQueue)
{
	boost::shared_ptr<QueryResultQueue> in = firstQueue;
	boost::shared_ptr<QueryResultQueue> out;
	// add blackboard segments for each predicate in the conjunction
	int counter = 1;
	int numFormulae = phi.formulae().size();
	for(const Formula &psi : phi.formulae()) {
		if(numFormulae == counter) {
			out = lastQueue;
		}
		else {
			out = boost::shared_ptr<QueryResultQueue>(new QueryResultQueue);
		}
		decompose(psi, in, out);
		// output queue of this predicate is used as input for the next one
		in = out;
		counter += 1;
	}
	// TODO: merge segments if possible, i.e. if a reasoner supports evaluation of conjunctive goals
}

void Blackboard::decomposeDisjunction(
		const DisjunctionFormula &phi,
		boost::shared_ptr<QueryResultQueue> &in,
		boost::shared_ptr<QueryResultQueue> &out)
{
	// add blackboard segments for each formula in the disjunction.
	// all having the same input and out queue.
	for(const Formula &psi : phi.formulae()) {
		decompose(psi, in, out);
	}
	// TODO: merge segments if possible, i.e. if a reasoner supports evaluation of disjunctive goals
}

void Blackboard::stopReasoningProcesses()
{
	for(auto& segment : segments_) {
		segment->stopReasoningProcesses();
	}
}

