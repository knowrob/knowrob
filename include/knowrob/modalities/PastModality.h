//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_PAST_MODALITY_H
#define KNOWROB_PAST_MODALITY_H

#include "TemporalModality.h"

namespace knowrob {
    /**
     * A time modality using operator "P" where `Pq` stands for "it is or was the case that q".
     * The operator "H" is the dual of "P" where `Hq` stands for "it is and was always the case that q".
     */
    class PastModality : public TemporalModality {
	protected:
		PastModality() : TemporalModality() {}

	public:
		/**
		 * @return the belief modality singleton.
		 */
		static const PastModality* get() {
			static PastModality instance;
			return &instance;
		}

		/**
		 * @return the belief operator `P`
		 */
		static const ModalOperator& P() {
			static const ModalOperator op(get(), ModalOperatorType::POSSIBILITY);
			return op;
		}

		/**
		 * @return the belief operator `H`
		 */
		static const ModalOperator& H() {
            static const ModalOperator op(get(), ModalOperatorType::NECESSITY);
			return op;
		}

        // Override Modality, modal axiom "D"
        // - note: assumes there is no begin of time
        //
        bool isSerial() const override { return true; }

        // Override Modality, modal axiom "T"
        // - note: assumes "current time" is included
        //
        bool isReflexive() const override { return true; }

        // Override Modality, modal axiom "4"
        // - all states reachable from the past are also directly reachable from "now".
        //
        bool isTransitive() const override { return true; }

        // Override Modality, modal axiom "C4"
        // - there is always a time instant between two others
        //
        bool isDense() const override { return true; }

        // Override Modality, modal axiom "5"
        // - cannot be adopted as we can only travel further into the past.
        //
        bool isEuclidean() const override { return false; }

        // Override Modality, modal axiom "B"
        // - cannot be adopted as we can only travel further into the past.
        //
        bool isSymmetric() const override { return false; }

        // Override EpistemicModality
        const char* necessity_symbol() const override { return "P"; }

        // Override EpistemicModality
        const char* possibility_symbol() const override { return "H"; }
    };

} // knowrob

#endif //KNOWROB_PAST_MODALITY_H
