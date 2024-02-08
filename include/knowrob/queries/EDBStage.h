//
// Created by daniel on 31.08.23.
//

#ifndef KNOWROB_EDB_STAGE_H
#define KNOWROB_EDB_STAGE_H

#include <memory>
#include "knowrob/db/KnowledgeGraph.h"
#include "QueryStageLiteral.h"

namespace knowrob {
	/**
	 * A query stage that runs an EDB query. It does so by attempting to ground
	 * literals that appear in the query in the extensional database.
	 */
	class EDBStage : public QueryStageLiteral {
	public:
		EDBStage(KnowledgeGraphPtr edb,
				 const RDFLiteralPtr &literal,
				 const QueryContextPtr &ctx);

	protected:
		KnowledgeGraphPtr edb_;

		TokenBufferPtr submitQuery(const RDFLiteralPtr &literal) override;
	};

} // knowrob

#endif //KNOWROB_EDB_STAGE_H
