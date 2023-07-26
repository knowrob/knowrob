//
// Created by daniel on 06.03.23.
//

#ifndef KNOWROB_ATOMIC_PROPOSITION_H
#define KNOWROB_ATOMIC_PROPOSITION_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/modalities/ModalFrame.h"
#include "knowrob/semweb/TripleData.h"

namespace knowrob {
	/**
	 * An atomic proposition is a statement or assertions that is thought
	 * to be true or false and without logical operators.
	 */
    class AtomicProposition {
    public:
        /**
         * @predicate a predicate without free variable.
         */
        explicit AtomicProposition(const PredicatePtr &predicate);

        const PredicatePtr& predicate() const { return predicate_; }

		/**
		 * @return the modal frame of this proposition.
		 */
		const ModalFrame& modalFrame() const { return modality_; }

		/**
		 * @param modality the modal frame of this proposition.
		 */
		void setModalFrame(const ModalFrame &modality) { modality_ = modality; }

    protected:
        const PredicatePtr predicate_;
		ModalFrame modality_;
    };

} // knowrob

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::AtomicProposition& s);
}

#endif //KNOWROB_ATOMIC_PROPOSITION_H
