//
// Created by daniel on 22.03.23.
//

#include "knowrob/terms/ModalOperator.h"

using namespace knowrob;

ModalOperator::ModalOperator(const Modality *modality, ModalOperatorType operatorType)
: Term(TermType::MODAL_OPERATOR),
  modality_(modality),
  operatorType_(operatorType)
{
}

bool ModalOperator::isTransitive() const
{
    return modality_->isTransitive();
}

bool ModalOperator::isEuclidean() const
{
    return modality_->isEuclidean();
}

const char* ModalOperator::symbol() const
{
    if(operatorType_ == ModalOperatorType::NECESSARY)
        return modality_->necessity_symbol();
    else
        return modality_->possibility_symbol();
}

void ModalOperator::write(std::ostream& os) const
{
    if(operatorType_ == ModalOperatorType::NECESSARY)
        os << modality_->necessity_symbol();
    else
        os << modality_->possibility_symbol();
}

bool ModalOperator::isEqual(const Term &other) const
{
    const auto &x = static_cast<const ModalOperator&>(other); // NOLINT
    return operatorType_ == x.operatorType_ && modality_ == x.modality_;
}
