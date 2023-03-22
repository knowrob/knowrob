//
// Created by daniel on 22.03.23.
//

#include "knowrob/terms/NestedModality.h"

using namespace knowrob;

void NestedModality::push_back(const ModalOperator &modalOperator)
{
    if(modalitySequence_.empty()) {
        modalitySequence_.push_back(modalOperator);
    }
    else {
        auto &last = modalitySequence_.back();
        if(last.modality().get() == modalOperator.modality().get()) {
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
