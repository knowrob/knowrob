/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/db/QueryableBackend.h"
#include "knowrob/ThreadPool.h"

using namespace knowrob;

TokenBufferPtr QueryableBackend::submitQuery(const ConjunctiveQueryPtr &query) {
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
