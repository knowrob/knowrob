/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reification/ReificationContainer.h"
#include "knowrob/reification/ReifiedTriple.h"

using namespace knowrob;

ReificationContainer::ReificationContainer(semweb::TripleContainerPtr originalTriples, semweb::VocabularyPtr vocabulary)
		: originalTriples_(std::move(originalTriples)), vocabulary_(std::move(vocabulary)) {
}

semweb::TripleContainer::ConstGenerator
getReifiedGenerator(const FramedTriple &triple, const semweb::VocabularyPtr &vocabulary) {
	auto reified = std::make_shared<ReifiedTriple>(triple, vocabulary);
	return [reified, it = reified->begin()]() mutable -> const FramedTriplePtr * {
		if (it == reified->end()) return nullptr;
		return &*it++;
	};
}

struct IterationData {
	IterationData(semweb::VocabularyPtr vocabulary, const semweb::TripleContainerPtr &originalTriples)
			: vocabulary(std::move(vocabulary)),
			  it(originalTriples->begin()),
			  end(originalTriples->end()),
			  reifiedGen(nullptr) {
	}
	semweb::VocabularyPtr vocabulary;
	semweb::TripleContainer::ConstGenerator reifiedGen;
	semweb::TripleContainer::iterator it;
	semweb::TripleContainer::iterator end;
};

semweb::TripleContainer::ConstGenerator ReificationContainer::cgenerator() const {
	auto data = std::make_shared<IterationData>(vocabulary_, originalTriples_);

	return [data]() mutable -> const FramedTriplePtr * {
		if (data->reifiedGen) {
			// if a reified triple is available, return it
			const FramedTriplePtr *nextReified = data->reifiedGen();
			if (nextReified) return nextReified;
			else data->reifiedGen = nullptr;
		}
		// else process the next triple from the original container
		if (data->it == data->end) return nullptr;
		const FramedTriplePtr *next = &*data->it++;
		if (ReifiedTriple::isReifiable(*next->ptr)) {
			data->reifiedGen = getReifiedGenerator(*next->ptr, data->vocabulary);
			return data->reifiedGen();
		} else {
			return next;
		}
	};
}
