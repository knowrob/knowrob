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
		PastModality();

	public:
		/**
		 * @return the past modality singleton.
		 */
		static const PastModality* get();

		/**
		 * @return the past operator `P`
		 */
		static const ModalOperatorPtr& P();

		/**
		 * @return the past operator `H`
		 */
		static const ModalOperatorPtr& H();

        // Override Modality
        bool isSerial() const override;

        // Override Modality
        bool isReflexive() const override;

        // Override Modality
        bool isTransitive() const override;

        // Override Modality
        bool isDense() const override;

        // Override Modality
        bool isEuclidean() const override;

        // Override Modality
        bool isSymmetric() const override;

        // Override Modality
        const char* necessity_symbol() const override;

        // Override Modality
        const char* possibility_symbol() const override;
    };

} // knowrob

#endif //KNOWROB_PAST_MODALITY_H