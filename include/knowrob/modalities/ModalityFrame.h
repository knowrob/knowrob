//
// Created by daniel on 17.07.23.
//

#ifndef KNOWROB_MODAL_FRAME_H
#define KNOWROB_MODAL_FRAME_H

#include "ModalOperator.h"
#include "TimeInterval.h"

namespace knowrob {
    class ModalityFrame {
    public:
        ModalityFrame();

        explicit ModalityFrame(const ModalIteration &modalIteration);

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
        ModalOperatorPtr epistemicOperator_;
        ModalOperatorPtr pastOperator_;

        bool isAboutPast() const;
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ModalityFrame& modalFrame);
}


#endif //KNOWROB_MODAL_FRAME_H
