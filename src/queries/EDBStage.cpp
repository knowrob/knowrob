#include "knowrob/queries/EDBStage.h"

#include <utility>

using namespace knowrob;

EDBStage::EDBStage(QueryableBackendPtr edb, const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx)
		: QueryStageLiteral(literal, ctx),
		  edb_(std::move(edb)) {
}

TokenBufferPtr EDBStage::submitQuery(const FramedTriplePatternPtr &literal) {
	return edb_->submitQuery(std::make_shared<GraphPathQuery>(literal, ctx_));
}
