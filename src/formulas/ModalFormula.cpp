//
// Created by daniel on 12.03.23.
//

#include <utility>

#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/PastModality.h"

using namespace knowrob;

ModalFormula::ModalFormula(const ModalOperatorPtr &modalOperator, const FormulaPtr &formula)
        : CompoundFormula(FormulaType::MODAL, { formula }),
          modalOperator_(modalOperator)
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
    return modalOperator_->isModalNecessity();
}

const char* ModalFormula::operator_symbol() const
{
    if(isModalNecessity()) {
        return modalOperator_->modality()->necessity_symbol();
    }
    else {
        return modalOperator_->modality()->possibility_symbol();
    }
}
