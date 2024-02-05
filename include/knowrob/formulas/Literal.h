//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_LITERAL_H
#define KNOWROB_LITERAL_H

#include <memory>
#include "Predicate.h"
#include "knowrob/semweb/StatementData.h"

namespace knowrob {
    /**
     * A literal is an atomic formula or its negation.
     */
    class Literal {
    public:
        Literal(const PredicatePtr &predicate, bool isNegative);

        /**
         * Substitution constructor.
         * @other a literal.
         * @sub a mapping from terms to variables.
         */
        Literal(const Literal &other, const Substitution &sub);

        /**
         * @return the predicate of this literal.
         */
        const auto& predicate() const { return predicate_; }

        /**
         * @return true if this is a negative literal.
         */
        auto isNegated() const { return isNegated_; }

        /**
         * Set the negated flag of this literal.
         * @param isNegated true indicates the literal is negated.
         */
        void setIsNegated(bool isNegated) { isNegated_ = isNegated; }

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
         * @return The number of variables contained in this literal.
         */
        virtual uint32_t numVariables() const { return predicate_->getVariables().size(); }

        /**
		 * Replaces variables in the literal with terms.
		 * @sub a substitution mapping.
		 * @return the created literal.
         */
        virtual std::shared_ptr<Literal> applySubstitution(const Substitution &sub) const
        { return std::make_shared<Literal>(*this, sub); }

        /**
         * Write the literal into an ostream.
         */
        virtual std::ostream& write(std::ostream& os) const;

    protected:
        const PredicatePtr predicate_;
        bool isNegated_;
    };

    using LiteralPtr = std::shared_ptr<Literal>;

} // knowrob

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::Literal& l);
}

#endif //KNOWROB_LITERAL_H
