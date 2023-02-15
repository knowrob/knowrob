/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Disjunction.h>

using namespace knowrob;

DisjunctionFormula::DisjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
: ConnectiveFormula(FormulaType::DISJUNCTION, formulae)
{
}

DisjunctionFormula::DisjunctionFormula(const DisjunctionFormula &other, const Substitution &sub)
: ConnectiveFormula(other, sub)
{
}

std::shared_ptr<Formula> DisjunctionFormula::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<DisjunctionFormula>(
		new DisjunctionFormula(*this, sub));
}
