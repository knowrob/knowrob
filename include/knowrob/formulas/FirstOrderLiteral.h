/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FIRST_ORDER_LITERAL_H
#define KNOWROB_FIRST_ORDER_LITERAL_H

#include <memory>
#include "Predicate.h"
#include "knowrob/triples/FramedTriple.h"

namespace knowrob {
	/**
	 * A FOL literal is an atomic formula or its negation.
	 */
	class FirstOrderLiteral {
	public:
		FirstOrderLiteral(const PredicatePtr &predicate, bool isNegative);

		/**
		 * @return the predicate of this literal.
		 */
		const auto &predicate() const { return predicate_; }

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
		auto &functor() const { return predicate_->functor(); }

		/**
		 * Get the arity of this predicate.
		 *
		 * @return arity of predicate
		 */
		auto arity() const { return predicate_->arity(); }

		/**
		 * @return The number of variables contained in this literal.
		 */
		virtual uint32_t numVariables() const { return predicate_->variables().size(); }

		/**
		 * Write the literal into an ostream.
		 */
		virtual std::ostream &write(std::ostream &os) const;

	protected:
		const PredicatePtr predicate_;
		bool isNegated_;
	};

	using FirstOrderLiteralPtr = std::shared_ptr<FirstOrderLiteral>;

} // knowrob

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::FirstOrderLiteral &l);
}

#endif //KNOWROB_FIRST_ORDER_LITERAL_H
