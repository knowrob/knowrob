/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Conjunction.h>

using namespace knowrob;

ConjunctionFormula::ConjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
: ConnectiveFormula(FormulaType::CONJUNCTION, formulae)
{
}

ConjunctionFormula::ConjunctionFormula(const ConjunctionFormula &other, const Substitution &sub)
: ConnectiveFormula(other, sub)
{
}

std::shared_ptr<Formula> ConjunctionFormula::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<ConjunctionFormula>(
		new ConjunctionFormula(*this, sub));
}
