//
// Created by daniel on 22.03.23.
//

#ifndef KNOWROB_NESTED_MODALITY_H
#define KNOWROB_NESTED_MODALITY_H

#include <memory>
#include <list>
#include "knowrob/modalities/Modality.h"
#include "knowrob/terms/ModalOperator.h"

namespace knowrob {
    /**
     * Represents an iteration over modal operators.
     */
    class NestedModality {
    public:
        NestedModality() = default;

        void push_back(const ModalOperator &modalOperator);

        auto begin() const { return modalitySequence_.begin(); }

        auto end() const { return modalitySequence_.end(); }

    protected:
        std::list<ModalOperator> modalitySequence_;
    };

    using ModalitySequencePtr = std::shared_ptr<NestedModality>;

} // knowrob

#endif //KNOWROB_NESTED_MODALITY_H
