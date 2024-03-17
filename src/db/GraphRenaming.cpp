/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/GraphRenaming.h"
#include <utility>

using namespace knowrob;

GraphRenaming::GraphRenaming(GraphRenamingMap renaming)
		: renaming_(std::move(renaming)) {
}

void GraphRenaming::addRenaming(std::string_view from, std::string_view to) {
	renaming_.emplace(std::string(from), std::string(to));
}

bool GraphRenaming::configure(const boost::property_tree::ptree &opts) {
	// TODO: support some file formats for loading key-value pairs
	return true;
}

std::string_view GraphRenaming::rename(const std::string_view &entity) {
	auto it = renaming_.find(entity);
	if (it != renaming_.end()) {
		return it->second;
	}
	return entity;
}

void GraphRenaming::rename(FramedTriple &triple) {
	triple.setSubject(rename(triple.subject()));
	triple.setPredicate(rename(triple.predicate()));
	if (triple.isObjectIRI()) {
		triple.setObjectIRI(rename(triple.valueAsString()));
	}
}

void GraphRenaming::initializeTransformation() {
	initializeNext();
}

void GraphRenaming::finalizeTransformation() {
	finalizeNext();
}

void GraphRenaming::pushInputTriples(const TripleContainerPtr &triples) {
	if (triples->isMutable()) {
		auto mutableTriples = std::static_pointer_cast<MutableTripleContainer>(triples);
		for (auto it = mutableTriples->begin(); it != mutableTriples->end(); ++it) {
			rename(**it);
		}
	} else {
		KB_WARN("Input triples are not mutable which is currently required for renaming.");
	}
	pushOutputTriples(triples);
}
