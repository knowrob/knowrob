/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/QueryStageLiteral.h"

using namespace knowrob;

QueryStageLiteral::QueryStageLiteral(RDFLiteralPtr literal, const QueryContextPtr &ctx)
		: QueryStage(ctx),
		  literal_(std::move(literal)) {
}

TokenBufferPtr QueryStageLiteral::submitQuery(const Substitution &substitution) {
	// apply the substitution mapping
	auto literalInstance = std::make_shared<FramedTriplePattern>(*literal_, substitution);
	// submit a query
	return submitQuery(literalInstance);
}
