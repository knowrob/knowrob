/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/reification/ReificationContainer.h"
#include "knowrob/reification/ReifiedTriple.h"

using namespace knowrob;

ReificationContainer::ReificationContainer(TripleContainerPtr originalTriples,
										   VocabularyPtr vocabulary,
										   ReifiedNames reifiedNames)
		: originalTriples_(std::move(originalTriples)),
		  vocabulary_(std::move(vocabulary)),
		  reifiedNames_(std::move(reifiedNames)) {
}

TripleContainer::ConstGenerator
getReifiedGenerator(const FramedTriple &triple,
					const VocabularyPtr &vocabulary,
					const ReifiedNames &reifiedNames,
					uint32_t tripleIndex) {
	std::shared_ptr<ReifiedTriple> reified;
	if (reifiedNames && !reifiedNames->empty()) {
		reified = std::make_shared<ReifiedTriple>(triple, vocabulary, (*reifiedNames)[tripleIndex - 1]);
	} else {
		reified = std::make_shared<ReifiedTriple>(triple, vocabulary);
	}
	return [reified, it = reified->begin()]() mutable -> const FramedTriplePtr * {
		if (it == reified->end()) return nullptr;
		return &*it++;
	};
}

namespace knowrob::reification {
	struct IterationData {
		IterationData(VocabularyPtr vocabulary, const TripleContainerPtr &originalTriples,
					  ReifiedNames reifiedNames)
				: vocabulary(std::move(vocabulary)),
				  it(originalTriples->begin()),
				  end(originalTriples->end()),
				  reifiedNames(std::move(reifiedNames)),
				  reifiedGen(nullptr),
				  tripleIndex(0) {
		}

		VocabularyPtr vocabulary;
		TripleContainer::ConstGenerator reifiedGen;
		TripleContainer::iterator it;
		TripleContainer::iterator end;
		ReifiedNames reifiedNames;
		uint32_t tripleIndex;
	};
}

TripleContainer::ConstGenerator ReificationContainer::cgenerator() const {
	auto data = std::make_shared<reification::IterationData>(
			vocabulary_, originalTriples_, reifiedNames_);

	return [data]() mutable -> const FramedTriplePtr * {
		if (data->reifiedGen) {
			// if a reified triple is available, return it
			const FramedTriplePtr *nextReified = data->reifiedGen();
			if (nextReified) return nextReified;
			else data->reifiedGen = nullptr;
		}
		// else process the next triple from the original container
		if (data->it == data->end) return nullptr;
		data->tripleIndex += 1;
		const FramedTriplePtr *next = &*data->it++;
		if (ReifiedTriple::isReifiable(*next->ptr)) {
			data->reifiedGen = getReifiedGenerator(*next->ptr, data->vocabulary, data->reifiedNames, data->tripleIndex);
			return data->reifiedGen();
		} else {
			return next;
		}
	};
}
