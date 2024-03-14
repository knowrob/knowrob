/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/QueryStageLiteral.h"

using namespace knowrob;

QueryStageLiteral::QueryStageLiteral(FramedTriplePatternPtr literal, const QueryContextPtr &ctx)
		: QueryStage(ctx),
		  literal_(std::move(literal)) {
}

TokenBufferPtr QueryStageLiteral::submitQuery(const Bindings &substitution) {
	// apply the substitution mapping
	auto literalInstance = applyBindings(literal_, substitution);
	// submit a query
	return submitQuery(literalInstance);
}
