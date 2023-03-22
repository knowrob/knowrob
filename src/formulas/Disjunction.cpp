/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Disjunction.h>

using namespace knowrob;

Disjunction::Disjunction(const std::vector<std::shared_ptr<Formula>> &formulae)
: CompoundFormula(FormulaType::DISJUNCTION, formulae)
{
}

Disjunction::Disjunction(const Disjunction &other, const Substitution &sub)
: CompoundFormula(other, sub)
{
}

std::shared_ptr<Formula> Disjunction::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Disjunction>(
		new Disjunction(*this, sub));
}
