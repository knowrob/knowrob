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
	public:
        KnowledgeModality();

        explicit KnowledgeModality(const std::string_view &agent);

		/**
		 * @return the knowledge operator `K`
		 */
		static ModalOperatorPtr K();

		/**
		 * @return the knowledge operator `K`
		 */
		static ModalOperatorPtr K(const std::string_view &agent);

        // Override Modality
        bool isSerial() const override;

        // Override Modality
        bool isReflexive() const override;

        // Override Modality
        bool isTransitive() const override;

        // Override Modality
        bool isEuclidean() const override;

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

#endif //KNOWROB_KNOWLEDGE_MODALITY_H
