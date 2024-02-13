/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/stacktrace.hpp>
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/ReasonerError.h"

using namespace knowrob;

Reasoner::Reasoner()
		: reasonerManagerID_(0) {
}

void Reasoner::setReasonerManager(uint32_t managerID) {
	reasonerManagerID_ = managerID;
}

void Reasoner::setReasonerName(std::string_view name) {
	t_reasonerName_ = std::make_shared<StringTerm>(name.data());
}

KnowledgeBase *Reasoner::kb() const {
	auto rm = ReasonerManager::getReasonerManager(reasonerManagerID_);
	if (rm) {
		return rm->kb();
	} else {
		throw ReasonerError("KnowledgeBase not found.", boost::stacktrace::stacktrace());
	}
}

void Reasoner::pushWork(const std::function<void(void)> &fn) {
	auto runner = std::make_shared<ThreadPool::LambdaRunner>(fn);
	DefaultThreadPool()->pushWork(runner, [](const std::exception &e) {
		KB_ERROR("Error in reasoner worker thread: {}", e.what());
	});
}


void Reasoner::initTriple(StatementData *triple) const {
	triple->graph = reasonerName().c_str();
	// TODO: also set other parameters of the triple frame.
}


StatementData Reasoner::createTriple() const {
	StatementData triple;
	initTriple(&triple);
	return triple;
}

std::vector<StatementData> Reasoner::createTriples(uint32_t count) const {
	std::vector<StatementData> triple(count);
	for (uint32_t i = 0; i < count; i++) {
		initTriple(&triple[i]);
	}
	return triple;
}

void Reasoner::setInferredTriples(const std::vector<StatementData> &triples) {
	if (inferredTriples_.empty()) {
		inferredTriples_.insert(triples.begin(), triples.end());
		addInferredTriples(triples);
	} else {
		auto &oldTriples = inferredTriples_;
		// ensure that input triples are sorted which is required for set_difference
		std::set<StatementData> newTriples(triples.begin(), triples.end());
		std::vector<StatementData> triplesToRemove, triplesToAdd;
		// old inferences without new inferences are the ones that do not hold anymore.
		std::set_difference(oldTriples.begin(), oldTriples.end(),
							newTriples.begin(), newTriples.end(),
							std::inserter(triplesToRemove, triplesToRemove.begin()));
		// new inferences without old inferences are the ones that are really new.
		std::set_difference(newTriples.begin(), newTriples.end(),
							oldTriples.begin(), oldTriples.end(),
							std::inserter(triplesToAdd, triplesToAdd.begin()));
		// update the set of inferred triples.
		for (auto &triple: triplesToRemove) {
			inferredTriples_.erase(triple);
		}
		inferredTriples_.insert(triplesToAdd.begin(), triplesToAdd.end());
		// update the knowledge base
		addInferredTriples(triplesToAdd);
		removeInferredTriples(triplesToAdd);
	}
}

void Reasoner::addInferredTriples(const std::vector<StatementData> &triples) const {
	kb()->insertAll(triples);
}

void Reasoner::removeInferredTriples(const std::vector<StatementData> &triples) const {
	kb()->removeAll(triples);
}

PredicateDescriptionPtr Reasoner::getLiteralDescription(const RDFLiteral &literal) {
	if (literal.propertyTerm()->type() == TermType::STRING) {
		auto p = std::static_pointer_cast<StringTerm>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->value(), 2));
	} else {
		return {};
	}
}
