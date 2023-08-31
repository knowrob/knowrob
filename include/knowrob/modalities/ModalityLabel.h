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
#include "knowrob/semweb/StatementData.h"
#include "EpistemicModality.h"
#include "TemporalModality.h"
#include "PastModality.h"

namespace knowrob {
    /**
     * Labels a formula by an iteration over modal operators.
     */
    class ModalityLabel : public FormulaLabel {
    public:
        explicit ModalityLabel(const ModalIteration &modalOperators);

        explicit ModalityLabel(const StatementData &tripleData);

        static std::shared_ptr<ModalityLabel> emptyLabel();

        /**
         * @param other another modalFrame label.
         * @return true this is the same label of other.
         */
        bool operator==(const ModalityLabel &other) const;

        /**
         * @return number of modal operators in this label.
         */
        //auto numOperators() const { return modalOperators_->numOperators(); }

        /**
         * @return an iteration over modal operators.
         */
        //const auto& modalOperators() const { return *modalOperators_; }

        /**
         * @return begin iterator of modal iteration.
         */
        //auto begin() const { return modalOperators_->begin(); }

        /**
         * @return end iterator of modal iteration.
         */
        //auto end() const { return modalOperators_->end(); }

		bool hasValue() const;

        bool isAboutKnowledge() const;

        bool isAboutBelief() const;

        bool isAboutPresent() const;

        bool isAboutSomePast() const;

        bool isAboutAllPast() const;

        const std::optional<std::string>& agent() const;

        const std::optional<TimeInterval>& timeInterval() const;

		void setTimeInterval(const TimeInterval &ti);

        ModalOperatorPtr epistemicOperator() const { return epistemicOperator_; }

        ModalOperatorPtr pastOperator() const { return pastOperator_; }

    protected:
        //const std::shared_ptr<ModalIteration> modalOperators_;
        ModalOperatorPtr epistemicOperator_;
        ModalOperatorPtr pastOperator_;

        bool isAboutPast() const;

        bool isEqual(const FormulaLabel &other) const;
    };

    using ModalityLabelPtr = std::shared_ptr<ModalityLabel>;

} // knowrob

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ModalityLabel& label);
}

#endif //KNOWROB_MODALITY_LABEL_H
