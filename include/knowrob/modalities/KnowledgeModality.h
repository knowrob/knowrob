//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_KNOWLEDGE_MODALITY_H
#define KNOWROB_KNOWLEDGE_MODALITY_H

#include "EpistemicModality.h"

namespace knowrob {
    /**
     * A modality using the operator "K" where `Kp` stands for "the agent knows that p".
     */
    class KnowledgeModality : public EpistemicModality {
	protected:
		KnowledgeModality() : EpistemicModality() {}

	public:
		/**
		 * @return the belief modality singleton.
		 */
		static const KnowledgeModality* get() {
			static KnowledgeModality instance;
			return &instance;
		}

		/**
		 * @return the belief operator `K`
		 */
		static const ModalOperator& K() {
			const ModalOperator op(get(), ModalOperatorType::NECESSARY);
			return op;
		}

        // Override Modality, modal axiom "D"
        //  - you never stop learning new knowledge, i.e. `K phi -> diamond_K phi`.
        //    so there will always be a possible world where the agent knows additional things.
        //
        bool isSerial() const { return true; }

        // Override Modality, modal axiom "T"
        // - what is known is considered to be true `K phi -> phi`.
        //
        bool isReflexive() const { return true; }

        // Override Modality, modal axiom "4"
        //  - what is known remains known: `KK phi -> K phi`.
        //    (the knowledge modality is truth preserving)
        //  - "principles of positive introspection"
        //
        bool isTransitive() const { return true; }

        // Override Modality, modal axiom "5"
        //  - TODO: this is disputed, I would follow Hintikka's viewpoint and
        //          avoid closed world assumption. but that seems to entail
        //          symmetry must be dropped too :/
        //
        bool isEuclidean() const { return false; }

        // Override Modality, modal axiom "B"
        //  - TODO: explain
        //
        bool isSymmetric() const { return false; }

        // Override Modality, modal axiom "C4"
        // - cannot be adopted assuming there is no step between a world
        //   and adding atomic propositions to it.
        //
        bool isDense() const { return false; }

        // Override EpistemicModality
        const char* necessity_symbol() const override { return "K"; }

        // Override EpistemicModality
        const char* possibility_symbol() const override { return "\u22C4_K"; }

    };

} // knowrob

#endif //KNOWROB_KNOWLEDGE_MODALITY_H
