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

void GraphRenaming::rename(StatementData &triple) {
	triple.subject = rename(triple.subject).data();
	triple.predicate = rename(triple.predicate).data();
	if (triple.objectType == RDFType::RDF_RESOURCE) {
		triple.object = rename(triple.object).data();
	}
}

void GraphRenaming::initializeTransformation() {
	initializeNext();
}

void GraphRenaming::finalizeTransformation() {
	finalizeNext();
}

void GraphRenaming::pushInputTriples(const semweb::MutableTripleContainerPtr &triples) {
	for (auto it = triples->beginMutable(); it != triples->endMutable(); ++it) {
		rename(*it);
	}
	pushOutputTriples(triples);
}
