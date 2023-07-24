//
// Created by daniel on 22.03.23.
//

#include "knowrob/modalities/ModalOperator.h"
#include "knowrob/modalities/Modality.h"

using namespace knowrob;

ModalOperator::ModalOperator(const std::shared_ptr<Modality> &modality, ModalOperatorType operatorType)
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
    // TODO: also include parameters of operator such as agent
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

bool ModalOperator::isModalNecessity() const
{
    return operatorType_ == ModalOperatorType::NECESSITY;
}

bool ModalOperator::isModalPossibility() const
{
    return operatorType_ == ModalOperatorType::POSSIBILITY;
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

void ModalIteration::operator+=(const ModalOperatorPtr &next) //NOLINT
{
    if(modalitySequence_.empty()) {
        modalitySequence_.push_back(next);
    }
    else {
        auto &last = modalitySequence_.back();

        ModalOperatorPtr reduced;
        if(last->modality().modalityType() == next->modality().modalityType()) {
            // Axiom (4): <square>p -> <square><square>p corresponds to a transitive accessibility relation.
            // If the axioms is adopted by a modality, then iteration over the same operator
            // symbol can be omitted.
            if(last->operatorType() == next->operatorType() &&
               last->modality().isTransitive())
            { reduced = last; }

            // Axiom (5): <square>p -> <diamond><square>p corresponds to a euclidean accessibility relation.
            // If the axioms is adopted by a modality, then iteration over possibility and necessity operator
            // can be reduced to the latter operator.
            if(last->operatorType() != next->operatorType() &&
               last->modality().isEuclidean())
            { reduced = next; }
        }
        if(!reduced) {
            // different modalities can be reduced in case some principles governing their
            // interaction or known.
            reduced = last->modality().reduce(last, next);
        }

        if(reduced) {
            modalitySequence_.pop_back();
            *this += reduced;
        }
        else {
            modalitySequence_.push_back(next);
        }
    }
}
