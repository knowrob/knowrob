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
        explicit KnowledgeModality(const std::optional<std::string> &agent={});

	public:
		/**
		 * @return the knowledge modality singleton.
		 */
		static const KnowledgeModality* get();

		/**
		 * @return the knowledge operator `K`
		 */
		static const ModalOperatorPtr& K();

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
