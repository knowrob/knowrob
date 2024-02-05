#include "knowrob/queries/EDBStage.h"

#include <utility>

using namespace knowrob;

EDBStage::EDBStage(KnowledgeGraphPtr edb, const RDFLiteralPtr &literal, const QueryContextPtr &ctx)
		: QueryStageLiteral(literal, ctx),
		  edb_(std::move(edb)) {
}

TokenBufferPtr EDBStage::submitQuery(const RDFLiteralPtr &literal) {
	return edb_->submitQuery(std::make_shared<ConjunctiveQuery>(literal, ctx_));
}
