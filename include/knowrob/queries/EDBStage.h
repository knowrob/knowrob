//
// Created by daniel on 31.08.23.
//

#ifndef KNOWROB_EDB_STAGE_H
#define KNOWROB_EDB_STAGE_H

#include <memory>
#include "knowrob/backend/QueryableBackend.h"
#include "QueryStageLiteral.h"
#include "knowrob/backend/BackendInterface.h"

namespace knowrob {
	/**
	 * A query stage that runs an EDB query. It does so by attempting to ground
	 * literals that appear in the query in the extensional database.
	 */
	class EDBStage : public QueryStageLiteral {
	public:
		EDBStage(TransactionCtrlPtr transactionCtrl,
				 QueryableBackendPtr edb,
				 const FramedTriplePatternPtr &literal,
				 const QueryContextPtr &ctx);

	protected:
		TransactionCtrlPtr transactionCtrl_;
		QueryableBackendPtr edb_;

		TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal) override;
	};

} // knowrob

#endif //KNOWROB_EDB_STAGE_H
