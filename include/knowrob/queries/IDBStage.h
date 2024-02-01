//
// Created by daniel on 31.08.23.
//

#ifndef KNOWROB_IDB_STAGE_H
#define KNOWROB_IDB_STAGE_H

#include "QueryStageLiteral.h"
#include "knowrob/reasoner/Reasoner.h"

namespace knowrob {
	/**
	 * A query stage that runs an IDB query. It does so by attempting to ground
	 * literals that appear in the query through a top-down method that perform the
	 * grounding on the fly through some form of computation.
	 */
	class IDBStage : public QueryStageLiteral {
	public:
		IDBStage(const std::shared_ptr<Reasoner> &reasoner,
				 const RDFLiteralPtr &literal,
				 const std::shared_ptr<ThreadPool> &threadPool,
				 const QueryContextPtr &ctx);

	protected:
		std::shared_ptr<Reasoner> reasoner_;
		std::shared_ptr<ThreadPool> threadPool_;

		TokenBufferPtr submitQuery(const RDFLiteralPtr &literal) override;
	};

} // knowrob

#endif //KNOWROB_IDB_STAGE_H
