//
// Created by daniel on 22.03.23.
//

#ifndef KNOWROB_MODALITY_LABEL_H
#define KNOWROB_MODALITY_LABEL_H

#include <memory>
#include <list>
#include "knowrob/modalities/Modality.h"
#include "ModalOperator.h"
#include "knowrob/formulas/Formula.h"
#include "EpistemicModality.h"
#include "TemporalModality.h"
#include "PastModality.h"

namespace knowrob {
    /**
     * Labels a formula by an iteration over modal operators.
     */
    class ModalityLabel : public FormulaLabel {
    public:
        explicit ModalityLabel(const std::shared_ptr<ModalIteration> &modalOperators)
        : FormulaLabel(), modalOperators_(modalOperators) {}

        /**
         * @param other another modalFrame label.
         * @return true this is the same label of other.
         */
        bool operator==(const ModalityLabel &other) const
        { return (this==&other) || (*modalOperators_ == *other.modalOperators_); }

        /**
         * @return number of modal operators in this label.
         */
        auto numOperators() const { return modalOperators_->numOperators(); }

        /**
         * @return an iteration over modal operators.
         */
        const auto& modalOperators() const { return *modalOperators_; }

        /**
         * @return begin iterator of modal iteration.
         */
        auto begin() const { return modalOperators_->begin(); }

        /**
         * @return end iterator of modal iteration.
         */
        auto end() const { return modalOperators_->end(); }

    protected:
        const std::shared_ptr<ModalIteration> modalOperators_;

        bool isEqual(const FormulaLabel &other) const override
        { return *this == *static_cast<const ModalityLabel*>(&other); } // NOLINT
    };

    using ModalityLabelPtr = std::shared_ptr<ModalityLabel>;

} // knowrob

#endif //KNOWROB_MODALITY_LABEL_H
