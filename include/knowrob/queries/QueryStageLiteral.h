/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_STAGE_LITERAL_H
#define KNOWROB_QUERY_STAGE_LITERAL_H

#include "QueryStage.h"
#include "knowrob/semweb/RDFLiteral.h"

namespace knowrob {
	class QueryStageLiteral : public QueryStage {
	public:
		QueryStageLiteral(RDFLiteralPtr literal, const QueryContextPtr &ctx);

	protected:
		/**
		 * Submits a query using given literal.
		 * @param literal a literal.
		 * @return an answer buffer.
		 */
		virtual TokenBufferPtr submitQuery(const RDFLiteralPtr &literal) = 0;

	protected:
		const RDFLiteralPtr literal_;

		// override QueryStage
		TokenBufferPtr submitQuery(const Substitution &substitution) override;
	};
} // knowrob

#endif //KNOWROB_QUERY_STAGE_LITERAL_H
