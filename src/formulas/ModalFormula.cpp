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

bool ModalFormula::isEqual(const Formula &other) const
{
	const auto &x = static_cast<const ModalFormula&>(other); // NOLINT
	return (*modalOperator()) == (*x.modalOperator()) &&
	       (*modalFormula()) == (*x.modalFormula());
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
        return modalOperator_->modality().necessity_symbol();
    }
    else {
        return modalOperator_->modality().possibility_symbol();
    }
}

void ModalFormula::write(std::ostream& os) const
{
    os << *modalOperator_ << ' ';
    os << *(formulae_[0].get());
}

namespace knowrob::modality {
    std::shared_ptr<ModalFormula> B(const FormulaPtr &phi)
    {
        return std::make_shared<ModalFormula>(BeliefModality::B(), phi);
    }

    std::shared_ptr<ModalFormula> K(const FormulaPtr &phi)
    {
        return std::make_shared<ModalFormula>(KnowledgeModality::K(), phi);
    }

    std::shared_ptr<ModalFormula> P(const FormulaPtr &phi)
    {
        return std::make_shared<ModalFormula>(PastModality::P(), phi);
    }

    std::shared_ptr<ModalFormula> H(const FormulaPtr &phi)
    {
        return std::make_shared<ModalFormula>(PastModality::H(), phi);
    }
} // knowrob::modality
