//
// Created by daniel on 22.03.23.
//

#ifndef KNOWROB_IMPLICATION_H
#define KNOWROB_IMPLICATION_H

#include "CompoundFormula.h"

namespace knowrob {

    class Implication : public CompoundFormula {
    public:
        explicit Implication(const FormulaPtr &antecedent, const FormulaPtr &consequent);

        const FormulaPtr& antecedent() const { return formulae_[0]; }

        const FormulaPtr& consequent() const { return formulae_[1]; }

        // Override Formula
        FormulaPtr applySubstitution(const Substitution &sub) const override;

        // Override ConnectiveFormula
        const char* operator_symbol() const override { return "->"; }

    protected:
        Implication(const Implication &other, const Substitution &sub);
    };

} // knowrob

#endif //KNOWROB_IMPLICATION_H
