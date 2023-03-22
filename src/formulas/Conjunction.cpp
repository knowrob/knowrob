/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Conjunction.h>

using namespace knowrob;

Conjunction::Conjunction(const std::vector<std::shared_ptr<Formula>> &formulae)
: CompoundFormula(FormulaType::CONJUNCTION, formulae)
{
}

Conjunction::Conjunction(const Conjunction &other, const Substitution &sub)
: CompoundFormula(other, sub)
{
}

std::shared_ptr<Formula> Conjunction::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Conjunction>(
		new Conjunction(*this, sub));
}
