//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_BELIEF_MODALITY_H
#define KNOWROB_BELIEF_MODALITY_H

#include "EpistemicModality.h"

namespace knowrob {
    /**
     * A modality using the operator "B" where `Bp` stands for "the agent believes that p".
     */
    class BeliefModality : public EpistemicModality {
	protected:
		explicit BeliefModality(const std::optional<std::string> &agent={});

    public:
        // Override Modality
        bool isSerial() const override;

        // Override Modality
        bool isTransitive() const override;

        // Override Modality
        bool isEuclidean() const override;

        // Override Modality
        bool isReflexive() const override;

        // Override Modality
        bool isSymmetric() const override;

        // Override Modality
        bool isDense() const override;

        // Override Modality
        const char* necessity_symbol() const override;

        // Override Modality
        const char* possibility_symbol() const override;

        // Override Modality
        ModalOperatorPtr reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const override;
	};

} // knowrob

#endif //KNOWROB_BELIEF_MODALITY_H
