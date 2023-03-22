//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_MODAL_FORMULA_H
#define KNOWROB_MODAL_FORMULA_H

#include "CompoundFormula.h"
#include "knowrob/modalities/Modality.h"
#include "knowrob/terms/ModalOperator.h"

namespace knowrob {
    /**
     * A formula using a unary modal operator.
     */
    class ModalFormula : public CompoundFormula {
    public:
        ModalFormula(ModalOperator modalOperator, const FormulaPtr &formula);

        const ModalOperator& modalOperator() const { return modalOperator_; }

        const FormulaPtr& modalFormula() const { return formulae_[0]; }

        bool isModalPossibility() const{  return !isModalNecessity(); }

        bool isModalNecessity() const;

        // Override CompoundFormula
        const char* operator_symbol() const override;

        // Override Formula
        FormulaPtr applySubstitution(const Substitution &sub) const override;

    protected:
        const ModalOperator modalOperator_;

        ModalFormula(const ModalFormula &other, const Substitution &sub);
    };
}


#endif // KNOWROB_MODAL_FORMULA_H
