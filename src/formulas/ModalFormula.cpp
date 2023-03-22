//
// Created by daniel on 12.03.23.
//

#include <utility>

#include "knowrob/formulas/ModalFormula.h"

using namespace knowrob;

ModalFormula::ModalFormula(ModalOperator modalOperator, const FormulaPtr &formula)
        : CompoundFormula(FormulaType::MODAL, { formula }),
          modalOperator_(std::move(modalOperator))
{
}

ModalFormula::ModalFormula(const ModalFormula &other, const Substitution &sub)
        : CompoundFormula(other, sub),
          modalOperator_(other.modalOperator_)
{
}

std::shared_ptr<Formula> ModalFormula::applySubstitution(const Substitution &sub) const
{
    return std::shared_ptr<ModalFormula>(new ModalFormula(*this, sub));
}

bool ModalFormula::isModalNecessity() const
{
    return modalOperator_.modalOperatorType() == ModalOperatorType::NECESSARY;
}

const char* ModalFormula::operator_symbol() const
{
    if(isModalNecessity()) {
        return modalOperator_.modality()->necessity_symbol();
    }
    else {
        return modalOperator_.modality()->possibility_symbol();
    }
}
