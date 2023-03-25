//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_MODAL_LITERAL_H
#define KNOWROB_MODAL_LITERAL_H

#include <memory>
#include "knowrob/modalities/ModalityLabel.h"
#include "Predicate.h"

namespace knowrob {
    /**
     * A literal is an atomic formula or its negation.
     */
    class Literal {
    public:
        Literal(const PredicatePtr &predicate, bool isNegative)
                : predicate_(predicate), isNegative_(isNegative) {}

        /**
         * @return the predicate of this literal.
         */
        const auto& predicate() const { return predicate_; }

        /**
         * @return true if this is a negative literal.
         */
        auto isNegative() const { return isNegative_; }

        /**
         * @return true if this is a positive literal.
         */
        auto isPositive() const { return !isNegative_; }

        /**
         * Get the functor of this literal.
         *
         * @return the functor name.
         */
        auto& functor() const { return predicate_->indicator()->functor(); }

        /**
         * Get the arity of this predicate.
         *
         * @return arity of predicate
         */
        auto arity() const { return predicate_->indicator()->arity(); }

    protected:
        const PredicatePtr predicate_;
        const bool isNegative_;
    };

    /**
     * A literal and a label.
     */
    class LabeledLiteral : public Literal {
    public:
        LabeledLiteral(const FormulaLabelPtr &label, const PredicatePtr &predicate, bool isNegated)
        : Literal(predicate, isNegated), label_(label) {}

        /**
         * @return the label of this literal.
         */
        const auto& label() const { return label_; }

    protected:
        const FormulaLabelPtr label_;
    };

} // knowrob

#endif //KNOWROB_MODAL_LITERAL_H
