/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/PredicateFormula.h>

using namespace knowrob;

PredicateFormula::PredicateFormula(const std::shared_ptr<Predicate> &predicate)
: Formula(FormulaType::PREDICATE),
  predicate_(predicate)
{
}

bool PredicateFormula::isGround() const
{
	return predicate_->isGround();
}

std::shared_ptr<Formula> PredicateFormula::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Formula>(new PredicateFormula(
		predicate_->applySubstitution(sub)));
}

void PredicateFormula::write(std::ostream& os) const
{
	predicate_->write(os);
}
