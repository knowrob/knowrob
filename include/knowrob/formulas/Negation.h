//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_NEGATED_FORMULA_H
#define KNOWROB_NEGATED_FORMULA_H

#include "CompoundFormula.h"
#include "Predicate.h"

namespace knowrob {
    /**
     * A negated formula.
     */
    class Negation : public CompoundFormula {
    public:
        /**
         * @formula the negated formula.
         */
        explicit Negation(const FormulaPtr &formula);

        const FormulaPtr& negatedFormula() const { return formulae_[0]; }

        // Override Formula
        FormulaPtr applySubstitution(const Substitution &sub) const override;

        // Override ConnectiveFormula
        const char* operator_symbol() const override { return "~"; }

    protected:
        Negation(const Negation &other, const Substitution &sub);
		bool isEqual(const Formula &other) const override;
    };

    /**
     * Negate a formula.
     * @param phi a formula
     * @return negation of phi.
     */
    FormulaPtr operator~(const FormulaPtr& phi);

} // knowrob

#endif //KNOWROB_NEGATED_FORMULA_H
