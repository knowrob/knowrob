/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/alignment/GraphRenaming.h"
#include <utility>
#include <fstream>

using namespace knowrob;

#define GRAPH_RENAMING_SETTING_FILE "file"

GraphRenaming::GraphRenaming(GraphRenamingMap renaming)
		: renaming_(std::move(renaming)) {
}

void GraphRenaming::addRenaming(std::string_view from, std::string_view to) {
	renaming_.emplace(std::string(from), std::string(to));
}

bool GraphRenaming::configure(const boost::property_tree::ptree &opts) {
	auto o_user = opts.get_optional<std::string>(GRAPH_RENAMING_SETTING_FILE);
	bool status = true;
	if (o_user) {
		try {
			status = readFromFile(*o_user);
		} catch (const std::exception &e) {
			KB_WARN("Error reading renaming from file {}: {}", *o_user, e.what());
			status = false;
		}
	}
	return status;
}

bool GraphRenaming::readFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key, value;

        if (!(iss >> key >> value)) {
            KB_WARN("Error reading renaming from file {}: Unexpected file format.", filename);
            return false;
        }

        renaming_[key] = value;
    }
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
