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
    public:
        BeliefModality() : EpistemicModality() {}

        // Override Modality, modal axiom "D"
        //  - the agent never stops forming new beliefs, i.e. `B phi -> diamond_B phi`.
        //    so there will always be a possible world where the agent beliefs in additional things.
        //
        bool isSerial() const { return true; }

        // Override Modality, modal axiom "4"
        //  - what is believed remains believed: `BB phi -> B phi`.
        //    (the belief modality is truth preserving)
        // - Note: mix instead with time modality to handle dynamics of beliefs.
        //
        bool isTransitive() const { return true; }

        // Override Modality, modal axiom "5"
        //  - TODO: this is disputed, I would follow Hintikka's viewpoint and
        //          avoid closed world assumption.
        //
        bool isEuclidean() const { return false; }

        // Override Modality, modal axiom "T"
        // - cannot be adopted as reflexivity would imply that `B phi -> phi`
        //   but that an agent believes that phi does not mean it is actually true.
        //
        bool isReflexive() const { return false; }

        // Override Modality, modal axiom "B"
        // - cannot be adopted assuming belief is truth preserving as we cannot
        //   travel back to the world where some belief is removed.
        // - Note: mix instead with time modality to handle dynamics of beliefs.
        //
        bool isSymmetric() const { return false; }

        // Override Modality, modal axiom "C4"
        // - cannot be adopted assuming there is no step between a world
        //   and adding an atomic belief to it.
        //
        bool isDense() const { return false; }

        // Override EpistemicModality
        const char* necessity_symbol() const override { return "B"; }

        // Override EpistemicModality
        const char* possibility_symbol() const override { return "\u22C4_B"; }
    };

} // knowrob

#endif //KNOWROB_BELIEF_MODALITY_H
