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
	t_reasonerName_ = Atom::Tabled(name);
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

InferredTripleContainer Reasoner::createTriples(uint32_t count) const {
	auto triples = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>(count);
	for (uint32_t i = 0; i < count; i++) {
		(*triples)[i] = std::make_shared<FramedTripleView>();
		(*triples)[i]->setGraph(reasonerName());
	}
	return triples;
}

bool Reasoner::InferredComparator::operator()(
		const std::shared_ptr<FramedTriple> &v0,
		const std::shared_ptr<FramedTriple> &v1) const {
	return *v0 < *v1;
}

void Reasoner::setInferredTriples(const InferredTripleContainer &triples) {
	if (inferredTriples_.empty()) {
		inferredTriples_.insert(triples->begin(), triples->end());
		addInferredTriples(triples);
	} else {
		auto &oldTriples = inferredTriples_;
		// ensure that input triples are sorted which is required for set_difference
		InferredeSet newTriples(triples->begin(), triples->end());

		auto triplesToRemove = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>();
		auto triplesToAdd = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>();
		// old inferences without new inferences are the ones that do not hold anymore.
		std::set_difference(oldTriples.begin(), oldTriples.end(),
							newTriples.begin(), newTriples.end(),
							std::inserter(*triplesToRemove, triplesToRemove->begin()));
		// new inferences without old inferences are the ones that are really new.
		std::set_difference(newTriples.begin(), newTriples.end(),
							oldTriples.begin(), oldTriples.end(),
							std::inserter(*triplesToAdd, triplesToAdd->begin()));
		// update the set of inferred triples.
		for (auto &triple: *triplesToRemove) {
			inferredTriples_.erase(triple);
		}
		inferredTriples_.insert(triplesToAdd->begin(), triplesToAdd->end());
		// update the knowledge base
		if (!triplesToAdd->empty()) {
			addInferredTriples(triplesToAdd);
		}
		if (!triplesToRemove->empty()) {
			removeInferredTriples(triplesToRemove);
		}
	}
}

void Reasoner::addInferredTriples(const InferredTripleContainer &triples) const {
	std::vector<FramedTriplePtr> triplesVector(triples->size());
	for (size_t i = 0; i < triples->size(); i++) {
		triplesVector[i].ptr = (*triples)[i].get();
		triplesVector[i].owned = false;
	}
	kb()->insertAll(triplesVector);
}

void Reasoner::removeInferredTriples(const InferredTripleContainer &triples) const {
	std::vector<FramedTriplePtr> triplesVector(triples->size());
	for (size_t i = 0; i < triples->size(); i++) {
		triplesVector[i].ptr = (*triples)[i].get();
		triplesVector[i].owned = false;
	}
	kb()->removeAll(triplesVector);
}

PredicateDescriptionPtr Reasoner::getLiteralDescription(const FramedTriplePattern &literal) {
	if (literal.propertyTerm()->termType() == TermType::ATOMIC) {
		auto p = std::static_pointer_cast<Atomic>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->stringForm().data(), 2));
	} else {
		return {};
	}
}
