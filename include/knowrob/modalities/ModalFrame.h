//
// Created by daniel on 17.07.23.
//

#ifndef KNOWROB_MODAL_FRAME_H
#define KNOWROB_MODAL_FRAME_H

#include "ModalOperator.h"
#include "TimeInterval.h"

namespace knowrob {
    class ModalFrame {
    public:
        ModalFrame();

        explicit ModalFrame(const ModalIteration &modalIteration);

        bool isAboutKnowledge() const;

        bool isAboutBelief() const;

        bool isAboutPresent() const;

        bool isAboutSomePast() const;

        bool isAboutAllPast() const;

        const std::optional<std::string>& agent() const;

        const std::optional<TimeInterval>& timeInterval() const;

        ModalOperatorPtr epistemicOperator() const { return epistemicOperator_; }

        ModalOperatorPtr pastOperator() const { return pastOperator_; }

    protected:
        ModalOperatorPtr epistemicOperator_;
        ModalOperatorPtr pastOperator_;

        bool isAboutPast() const;
    };
}


#endif //KNOWROB_MODAL_FRAME_H
