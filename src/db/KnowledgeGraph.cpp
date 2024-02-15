/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <fstream>

#include "knowrob/Logger.h"
#include "knowrob/db/KnowledgeGraph.h"

using namespace knowrob;

KnowledgeGraph::KnowledgeGraph()
		: vocabulary_(std::make_shared<semweb::Vocabulary>()),
		  importHierarchy_(std::make_unique<semweb::ImportHierarchy>()) {
}

KnowledgeGraph::~KnowledgeGraph() {
	// FIXME: stop all GraphQueryRunner's as they hold a pointer to this
}

bool KnowledgeGraph::isDefinedResource(const std::string_view &iri) {
	return isDefinedClass(iri) || isDefinedProperty(iri);
}

bool KnowledgeGraph::isDefinedProperty(const std::string_view &iri) {
	return vocabulary_->isDefinedProperty(iri);
}

bool KnowledgeGraph::isDefinedClass(const std::string_view &iri) {
	return vocabulary_->isDefinedClass(iri);
}

TokenBufferPtr KnowledgeGraph::submitQuery(const ConjunctiveQueryPtr &query) {
	std::shared_ptr<TokenBuffer> result = std::make_shared<TokenBuffer>();
	auto runner =
			std::make_shared<ThreadPool::LambdaRunner>([this, query, result](const ThreadPool::LambdaRunner::StopChecker&) {
				evaluateQuery(query, result);
			});
	DefaultThreadPool()->pushWork(runner, [result, query](const std::exception &e) {
		KB_WARN("an exception occurred for graph query ({}): {}.", *query, e.what());
		result->close();
	});
	return result;
}
