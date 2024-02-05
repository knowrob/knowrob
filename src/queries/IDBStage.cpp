#include <utility>

#include "knowrob/queries/IDBStage.h"

using namespace knowrob;

IDBStage::IDBStage(
		const std::shared_ptr<Reasoner> &reasoner,
		const RDFLiteralPtr &literal,
		const std::shared_ptr<ThreadPool> &threadPool,
		const QueryContextPtr &ctx)
		: QueryStageLiteral(literal, ctx),
		  reasoner_(reasoner),
		  threadPool_(threadPool) {
}

TokenBufferPtr IDBStage::submitQuery(const RDFLiteralPtr &literal) {
	return reasoner_->submitQuery(literal, ctx_);
}
