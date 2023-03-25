//
// Created by daniel on 22.03.23.
//

#include "knowrob/modalities/ModalOperator.h"

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
    if(operatorType_ == ModalOperatorType::NECESSITY)
        return modality_->necessity_symbol();
    else
        return modality_->possibility_symbol();
}

void ModalOperator::write(std::ostream& os) const
{
    if(operatorType_ == ModalOperatorType::NECESSITY)
        os << modality_->necessity_symbol();
    else
        os << modality_->possibility_symbol();
}

bool ModalOperator::isEqual(const Term &other) const
{
    const auto &x = static_cast<const ModalOperator&>(other); // NOLINT
    return operatorType_ == x.operatorType_ && modality_ == x.modality_;
}

const std::shared_ptr<ModalIteration>& ModalIteration::emptyIteration()
{
    static auto empty = std::make_shared<ModalIteration>();
    return empty;
}

bool ModalIteration::operator==(const ModalIteration &other) const
{
    if(modalitySequence_.size() != other.modalitySequence_.size()) {
        return false;
    }
    auto it = modalitySequence_.begin();
    auto jt = other.modalitySequence_.begin();
    while(it != modalitySequence_.end()) {
        if(!(*it == *jt)) {
            return false;
        }
        ++it;
        ++jt;
    }
    return true;
}

void ModalIteration::push_back(const ModalOperator &modalOperator)
{
    if(modalitySequence_.empty()) {
        modalitySequence_.push_back(modalOperator);
    }
    else {
        auto &last = modalitySequence_.back();
        if(last.modality() == modalOperator.modality()) {
            // do some simplifications on the fly if the same modality is iterated
            if(last.modalOperatorType() == modalOperator.modalOperatorType()) {
                if(modalOperator.isTransitive()) {
                    // no need to add the same operator twice in case of transitivity
                    // note: an operation would be needed here for graded modalities,
                    //       e.g. "10m ago it was the case that 10m ago it was the case that ..."
                    //       becomes "20m ago it was the case that ...".
                    return;
                }
            }
            else if(modalOperator.isEuclidean()) {
                // iteration from possible to necessary or vice version can
                // be simplified to just using the latter operator.
                modalitySequence_.pop_back();
            }
        }
        modalitySequence_.push_back(modalOperator);
    }
}
