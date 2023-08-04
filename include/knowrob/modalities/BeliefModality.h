//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_BELIEF_MODALITY_H
#define KNOWROB_BELIEF_MODALITY_H

#include "EpistemicModality.h"

namespace knowrob {
    /**
     * A modalFrame using the operator "B" where `Bp` stands for "the agent believes that p".
     */
    class BeliefModality : public EpistemicModality {
	public:
        BeliefModality();

		explicit BeliefModality(const std::string_view &agent);

		explicit BeliefModality(double confidence);

        BeliefModality(const std::string_view &agent, double confidence);

        /**
		 * @return the belief operator `B`
		 */
		static ModalOperatorPtr B();

        /**
		 * @return the belief operator `B`
		 */
		static ModalOperatorPtr B(const std::string_view &agent);

        /**
		 * @return the belief operator `B`
		 */
		static ModalOperatorPtr B(const std::string_view &agent, double confidence);

        /**
		 * @return the belief operator `B`
		 */
		static ModalOperatorPtr B(double confidence);

        auto& confidence() const { return confidence_; }

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
        ModalOperatorPtr reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const override;

    protected:
        std::optional<double> confidence_;
	};

} // knowrob

#endif //KNOWROB_BELIEF_MODALITY_H
