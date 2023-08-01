//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_LITERAL_H
#define KNOWROB_LITERAL_H

#include <memory>
#include "knowrob/modalities/ModalityLabel.h"
#include "Predicate.h"
#include "knowrob/semweb/StatementData.h"

namespace knowrob {
    /**
     * A literal is an atomic formula or its negation.
     */
    class Literal {
    public:
        Literal(const PredicatePtr &predicate, bool isNegative)
                : predicate_(predicate), isNegative_(isNegative) {}
        /**
         * Substitution constructor.
         *
         * @other a literal.
         * @sub a mapping from terms to variables.
         */
        Literal(const Literal &other, const Substitution &sub)
                : predicate_(std::make_shared<Predicate>(*other.predicate_, sub)),
                  isNegative_(other.isNegative_) {}

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

        /**
		 * Replaces variables in the literal with terms.
		 * @sub a substitution mapping.
		 * @return the created literal.
         */
        auto applySubstitution(const Substitution &sub) const
        { return std::make_shared<Literal>(*this, sub); }

    protected:
        const PredicatePtr predicate_;
        const bool isNegative_;
    };

    /**
     * A literal and a label.
     */
    class LabeledLiteral : public Literal {
    public:
        LabeledLiteral(const ModalityLabelPtr &label, const PredicatePtr &predicate, bool isNegated)
        : Literal(predicate, isNegated), label_(label) {}

        /**
         * @return the label of this literal.
         */
        const auto& label() const { return label_; }

        StatementData asStatementData() const;

    protected:
        const ModalityLabelPtr label_;
    };

    using LiteralPtr = std::shared_ptr<Literal>;
    using LabeledLiteralPtr = std::shared_ptr<LabeledLiteral>;

} // knowrob

#endif //KNOWROB_LITERAL_H
