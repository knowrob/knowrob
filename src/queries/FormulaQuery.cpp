/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/FormulaQuery.h>
#include <knowrob/formulas/Predicate.h>

using namespace knowrob;

FormulaQuery::FormulaQuery(const std::shared_ptr<Formula> &formula, const QueryContextPtr &ctx)
: Query(ctx), formula_(formula)
{
}

std::shared_ptr<FormulaQuery> FormulaQuery::applySubstitution(const Substitution &sub) const
{
	return std::make_shared<FormulaQuery>(
		formula_->isGround() ?
		formula_ :
		applyBindings(formula_, sub),
		ctx_
	);
}

std::ostream& FormulaQuery::print(std::ostream &os) const
{
    os << *formula();
    return os;
}
