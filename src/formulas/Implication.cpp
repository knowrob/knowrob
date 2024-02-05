//
// Created by daniel on 22.03.23.
//

#include "knowrob/formulas/Implication.h"

using namespace knowrob;

Implication::Implication(const FormulaPtr &antecedent, const FormulaPtr &consequent)
        : CompoundFormula(FormulaType::IMPLICATION, { antecedent, consequent })
{
}

Implication::Implication(const Implication &other, const Substitution &sub)
        : CompoundFormula(other, sub)
{
}

bool Implication::isEqual(const Formula &other) const
{
	const auto &x = static_cast<const Implication&>(other); // NOLINT
	return (*antecedent()) == (*x.antecedent()) &&
	       (*consequent()) == (*x.consequent());
}

std::shared_ptr<Formula> Implication::applySubstitution(const Substitution &sub) const
{
    return std::shared_ptr<Implication>(
            new Implication(*this, sub));
}
