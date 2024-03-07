/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_STAGE_LITERAL_H
#define KNOWROB_QUERY_STAGE_LITERAL_H

#include "QueryStage.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob {
	class QueryStageLiteral : public QueryStage {
	public:
		QueryStageLiteral(FramedTriplePatternPtr literal, const QueryContextPtr &ctx);

	protected:
		/**
		 * Submits a query using given literal.
		 * @param literal a literal.
		 * @return an answer buffer.
		 */
		virtual TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal) = 0;

	protected:
		const FramedTriplePatternPtr literal_;

		// override QueryStage
		TokenBufferPtr submitQuery(const Substitution &substitution) override;
	};
} // knowrob

#endif //KNOWROB_QUERY_STAGE_LITERAL_H
