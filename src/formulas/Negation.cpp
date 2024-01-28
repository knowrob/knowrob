//
// Created by daniel on 21.03.23.
//

#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Bottom.h"

using namespace knowrob;

Negation::Negation(const FormulaPtr &formula)
        : CompoundFormula(FormulaType::NEGATION, { formula })
{
}

Negation::Negation(const Negation &other, const Substitution &sub)
        : CompoundFormula(other, sub)
{
}

bool Negation::isEqual(const Formula &other) const
{
	const auto &x = static_cast<const Negation&>(other); // NOLINT
	return (*negatedFormula()) == (*x.negatedFormula());
}

std::shared_ptr<Formula> Negation::applySubstitution(const Substitution &sub) const
{
    return std::shared_ptr<Negation>(new Negation(*this, sub));
}

namespace knowrob {
    FormulaPtr operator~(const FormulaPtr &phi) {
        if(phi->type() == FormulaType::NEGATION) {
            return ((Negation*)phi.get())->negatedFormula();
        }
        else if(phi->isBottom()) {
            return Top::get();
        }
        else if(phi->isTop()) {
            return Bottom::get();
        }
        else {
            return std::make_shared<Negation>(phi);
        }
    }
}
