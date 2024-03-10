/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_STAGE_FORMULA_H
#define KNOWROB_QUERY_STAGE_FORMULA_H

#include "QueryStage.h"

namespace knowrob {
	class QueryStageFormula : public QueryStage {
	public:
		QueryStageFormula(FormulaPtr formula, const QueryContextPtr &ctx);

	protected:
		/**
		 * Submits a query using given formula.
		 * @param formula a formula.
		 * @return an answer buffer.
		 */
		virtual TokenBufferPtr submitQuery(const FormulaPtr &formula) = 0;

	protected:
		const FormulaPtr formula_;

		TokenBufferPtr submitQuery(const Bindings &substitution) override;
	};
} // knowrob

#endif //KNOWROB_QUERY_STAGE_H
