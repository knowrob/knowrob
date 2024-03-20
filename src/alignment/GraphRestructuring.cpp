/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/alignment/GraphRestructuring.h"
#include "knowrob/triples/SPARQLQuery.h"
#include "knowrob/queries/QueryParser.h"
#include <fstream>

using namespace knowrob;

#define GRAPH_RESTRUCTURING_SETTING_FILE "file"

GraphRestructuring::GraphRestructuring()
		: GraphTransformation(),
		  model_(nullptr) {
}

void GraphRestructuring::addRule(std::shared_ptr<GraphTransformationRule> rule) {
	rules_.push_back(std::move(rule));
}

bool GraphRestructuring::configure(const boost::property_tree::ptree &opts) {
	auto o_user = opts.get_optional<std::string>(GRAPH_RESTRUCTURING_SETTING_FILE);
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

bool GraphRestructuring::readFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string line, entry;

    while (std::getline(file, line, '.')) {
        entry += line;
        if (file.peek() != '\n') continue;

		file.ignore();
		std::stringstream ss(entry);
		std::string part1, part2;

		// split the entry into two parts at the '<-' separator
		std::getline(ss, part1, '<');
		ss.ignore(2); // ignore the '-' and ' '
		std::getline(ss, part2);
		if (part1.size()<5 || part2.size()<5) {
			KB_WARN("Error reading transformation from file {}: Invalid Syntax.", filename);
			return false;
		}

		// remove the brackets
		part1 = part1.substr(1, part1.size() - 2);
		part2 = part2.substr(1, part2.size() - 2);

		std::vector<FramedTriplePatternPtr> x, y;
		std::stringstream ss1(part1), ss2(part2);
		std::string item;

		while (std::getline(ss1, item, ',')) {
			auto pat = readTriplePattern(item);
			if (pat) {
				x.push_back(pat);
			} else {
				KB_WARN("Error reading transformation from file {}: Invalid triple pattern \"{}\".", filename, item);
				return false;
			}
		}
		while (std::getline(ss2, item, ',')) {
			auto pat = readTriplePattern(item);
			if (pat) {
				y.push_back(pat);
			} else {
				KB_WARN("Error reading transformation from file {}: Invalid triple pattern \"{}\".", filename, item);
				return false;
			}
		}
		addRule(std::make_shared<GraphTransformationRule>(x, y));

		entry.clear();
    }

    return true;
}

FramedTriplePatternPtr GraphRestructuring::readTriplePattern(const std::string& stringForm) {
	auto p = QueryParser::parsePredicate(stringForm);
	if (!p) return nullptr;
	try {
		auto p_rdf = FramedTriplePattern::getRDFPredicate(p);
		return std::make_shared<FramedTriplePattern>(p_rdf);
	} catch (const std::exception &e) {
		KB_WARN("Error reading transformation from file: {}", e.what());
		return nullptr;
	}
}

void GraphRestructuring::initializeTransformation() {
	model_ = std::make_unique<RedlandModel>();
	model_->setStorageType(RedlandStorageType::MEMORY);
	model_->setOrigin(origin_);
}

void GraphRestructuring::finalizeTransformation() {
	// do the transformation after last triple has been received
	for (auto &rule: rules_) {
		doTransformation(*rule);
	}
	// push into next stage
	initializeNext();
	model_->batch([this](const TripleContainerPtr &triples) {
		pushOutputTriples(triples);
	});
	finalizeNext();
	model_ = nullptr;
}

void GraphRestructuring::pushInputTriples(const TripleContainerPtr &triples) {
	model_->insertAll(triples);
}

void GraphRestructuring::doTransformation(GraphTransformationRule &rule) {
	// collect matching originals, and apply the transformation by finding and applying
	// a substitution mapping to the pattern predicates.
	auto originals = std::make_shared<TriplePatternContainer>();
	auto transformed = std::make_shared<TriplePatternContainer>();
	// perform query and record transformations
	model_->query(rule.getSPARQLQuery(), [&](const BindingsPtr &bindings) {
		// apply the substitution mapping to the pattern term
		for (auto &p: rule.to()) {
			auto x = applyBindings(p, *bindings);
			x->setGraphName(origin_);
			transformed->push_back(x);
		}
		// apply the substitution mapping to the query term
		for (auto &p: rule.from()) {
			originals->push_back(applyBindings(p, *bindings));
		}
	});
	// update the model
	model_->removeAll(originals);
	model_->insertAll(transformed);
}
