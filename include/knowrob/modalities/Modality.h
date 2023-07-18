//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_MODALITY_H
#define KNOWROB_MODALITY_H

#include <memory>
#include "knowrob/modalities/ModalOperator.h"

namespace knowrob {
    enum class ModalityType {
        Epistemic,
        Temporal_Past
    };

    /**
     * A modality for interpreting the truth of statements.
     */
    class Modality {
    public:
        Modality() = default;

        bool isEquivalenceRelation() const { return isReflexive() && isSymmetric() && isTransitive(); }

        virtual ModalityType modalityType() const = 0;

        /**
         * Seriality corresponds to modal axiom "D": `square phi -> diamond phi`.
         * @return true if the accessibility relation of this modality is serial.
         */
        virtual bool isSerial() const = 0;

        /**
         * Reflexivity corresponds to modal axiom "T": `square phi -> phi`.
         * @return true if the accessibility relation of this modality is reflexive.
         */
        virtual bool isReflexive() const = 0;

        /**
         * Symmetry corresponds to modal axiom "B": `diamond square phi -> phi`.
         * @return true if the accessibility relation of this modality is symmetric.
         */
        virtual bool isSymmetric() const = 0;

        /**
         * Transitivity corresponds to modal axiom "4": `square phi -> square square phi`.
         * @return true if the accessibility relation of this modality is transitive.
         */
        virtual bool isTransitive() const = 0;

        /**
         * Density corresponds to modal axiom "C4": `square square phi -> square phi`.
         * @return true if the accessibility relation of this modality is dense.
         */
        virtual bool isDense() const = 0;

        /**
         * Euclidean corresponds to modal axiom "5": `square phi -> diamond square phi`.
         * @return true if the accessibility relation of this modality is euclidean.
         */
        virtual bool isEuclidean() const = 0;

        /**
         * @return operator symbol for modal necessity.
         */
        virtual const char* necessity_symbol() const = 0;

        /**
         * @return operator symbol for modal possibility.
         */
        virtual const char* possibility_symbol() const = 0;

        /**
         * Attempt to reduce the iteration over two modal operators.
         * @param a a modal operator
         * @param b a model operator
         * @return the iteration reduced into a single operator if possible
         */
        virtual ModalOperatorPtr reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const
        { return {}; }
    };

    using ModalityPtr = std::shared_ptr<Modality>;
} // knowrob

#endif //KNOWROB_MODALITY_H
