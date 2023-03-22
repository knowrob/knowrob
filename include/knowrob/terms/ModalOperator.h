//
// Created by daniel on 22.03.23.
//

#ifndef KNOWROB_MODAL_OPERATOR_H
#define KNOWROB_MODAL_OPERATOR_H

#include "Term.h"
#include "knowrob/modalities/Modality.h"

namespace knowrob {
    enum class ModalOperatorType {
        NECESSARY,
        POSSIBLE
    };

    class ModalOperator : public Term {
    public:
        ModalOperator(const ModalityPtr &modality, ModalOperatorType operatorType);

        const ModalityPtr& modality() const { return modality_; }

        ModalOperatorType modalOperatorType() const { return operatorType_; }

        bool isTransitive() const;

        bool isEuclidean() const;

        // Override Term
        bool isGround() const override { return true; }

        // Override Term
        bool isAtomic() const override { return true; }

        // Override Term
        const VariableSet& getVariables() override { return Term::noVariables_; }

        // Override Term
        void write(std::ostream& os) const override;

    protected:
        const ModalityPtr modality_;
        const ModalOperatorType operatorType_;

        // Override Term
        bool isEqual(const Term &other) const override;
    };

} // knowrob

#endif //KNOWROB_MODAL_OPERATOR_H
