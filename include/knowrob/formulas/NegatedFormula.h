//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_NEGATED_FORMULA_H
#define KNOWROB_NEGATED_FORMULA_H

#include "CompoundFormula.h"

namespace knowrob {
    /**
     * A negated formula.
     */
    class NegatedFormula : public CompoundFormula {
    public:
        /**
         * @formula the negated formula.
         */
        explicit NegatedFormula(const FormulaPtr &formula);

        const FormulaPtr& negatedFormula() const { return formulae_[0]; }

        // Override Formula
        FormulaPtr applySubstitution(const Substitution &sub) const override;

        // Override ConnectiveFormula
        const char* operator_symbol() const override { return "~"; }

    protected:
        NegatedFormula(const NegatedFormula &other, const Substitution &sub);
    };

} // knowrob

#endif //KNOWROB_NEGATED_FORMULA_H
