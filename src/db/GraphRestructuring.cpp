/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/GraphRestructuring.h"
#include "knowrob/triples/SPARQLQuery.h"

using namespace knowrob;

GraphRestructuring::GraphRestructuring()
		: GraphTransformation(),
		  model_(nullptr) {
}

void GraphRestructuring::addRule(std::shared_ptr<GraphTransformationRule> rule) {
	rules_.push_back(std::move(rule));
}

bool GraphRestructuring::configure(const boost::property_tree::ptree &opts) {
	// TODO: support some file formats for loading graph transformation rules
	return true;
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
