#include "knowrob/queries/EDBStage.h"

#include <utility>

using namespace knowrob;

EDBStage::EDBStage(TransactionCtrlPtr transactionCtrl, QueryableBackendPtr edb, const FramedTriplePatternPtr &literal,
				   const QueryContextPtr &ctx)
		: QueryStageLiteral(literal, ctx),
		  transactionCtrl_(std::move(transactionCtrl)),
		  edb_(std::move(edb)) {
}

TokenBufferPtr EDBStage::submitQuery(const FramedTriplePatternPtr &literal) {
	return transactionCtrl_->getAnswerCursor(edb_, std::make_shared<GraphPathQuery>(literal, ctx_));
}
