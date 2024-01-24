#include "knowrob/queries/EDBStage.h"

#include <utility>

using namespace knowrob;

EDBStage::EDBStage(KnowledgeGraphPtr edb, const RDFLiteralPtr &literal, const QueryContextPtr &ctx)
: LiteralQueryStage(literal, ctx),
  edb_(std::move(edb))
{
}

AnswerBufferPtr EDBStage::submitQuery(const RDFLiteralPtr &literal)
{
    return edb_->submitQuery(std::make_shared<GraphQuery>(literal, ctx_));
}
