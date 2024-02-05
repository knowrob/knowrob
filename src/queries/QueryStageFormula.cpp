/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/queries/QueryStageFormula.h"

using namespace knowrob;

QueryStageFormula::QueryStageFormula(FormulaPtr formula, const QueryContextPtr &ctx)
		: QueryStage(ctx),
		  formula_(std::move(formula)) {
}

TokenBufferPtr QueryStageFormula::submitQuery(const Substitution &substitution) {
	// apply the substitution mapping
	auto formulaInstance = formula_->applySubstitution(substitution);
	// submit a query
	return submitQuery(formulaInstance);
}
