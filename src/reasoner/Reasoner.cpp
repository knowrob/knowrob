/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/ReasonerError.h"

using namespace knowrob;

Reasoner::Reasoner()
		: reasonerManager_(nullptr) {
}

void Reasoner::setReasonerManager(ReasonerManager *reasonerManager) {
	reasonerManager_ = reasonerManager;
}

void Reasoner::setReasonerName(std::string_view name) {
	t_reasonerName_ = std::make_shared<StringTerm>(name.data());
}

KnowledgeBase *Reasoner::kb() const {
	return reasonerManager().kb();
}

ReasonerManager &Reasoner::reasonerManager() const {
	if (reasonerManager_) {
		return *reasonerManager_;
	} else {
		throw ReasonerError("ReasonerManager not found.");
	}
}

std::shared_ptr<semweb::Vocabulary> Reasoner::vocabulary() const {
	return reasonerManager().kb()->vocabulary();
}

std::shared_ptr<semweb::ImportHierarchy> Reasoner::importHierarchy() const {
	return reasonerManager().kb()->importHierarchy();
}

class ReasonerTask : public ThreadPool::Runner {
public:
	explicit ReasonerTask(const std::function<void()> &fn) : fn_(fn) {}
	void run() override { fn_(); }

	protected:
		std::function<void()> fn_;
};

void Reasoner::pushWork(const std::function<void(void)> &fn) {
	// TODO: add support for stop request flag. For this the lambda needs to take an additional parameter.
	//       which itself is a function that returns the stop request flag of the worker.
	auto runner = std::make_shared<ReasonerTask>(fn);
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

class ReasonerTripleContainer : public semweb::TripleContainer {
public:
	explicit ReasonerTripleContainer(const std::vector<StatementData> *triples) : triples_(triples) {}

	const std::vector<StatementData> &asVector() const override { return *triples_; }

protected:
	const std::vector<StatementData> *triples_;
};

void Reasoner::addInferredTriples(const std::vector<StatementData> &triples) const {
	// Note: insertAll blocks until the triples are inserted, so it is safe to use the triples vector as a pointer.
	kb()->insertAll(std::make_shared<ReasonerTripleContainer>(&triples));
}

void Reasoner::removeInferredTriples(const std::vector<StatementData> &triples) const {
	kb()->removeAll(std::make_shared<ReasonerTripleContainer>(&triples));
}

PredicateDescriptionPtr Reasoner::getLiteralDescription(const RDFLiteral &literal) {
	if (literal.propertyTerm()->type() == TermType::STRING) {
		auto p = std::static_pointer_cast<StringTerm>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->value(), 2));
	} else {
		return {};
	}
}
