//
// Created by daniel on 21.03.23.
//

#include "knowrob/formulas/NegatedFormula.h"

using namespace knowrob;

NegatedFormula::NegatedFormula(const FormulaPtr &formula)
        : CompoundFormula(FormulaType::IMPLICATION, { formula })
{
}

NegatedFormula::NegatedFormula(const NegatedFormula &other, const Substitution &sub)
        : CompoundFormula(other, sub)
{
}

std::shared_ptr<Formula> NegatedFormula::applySubstitution(const Substitution &sub) const
{
    return std::shared_ptr<NegatedFormula>(new NegatedFormula(*this, sub));
}
