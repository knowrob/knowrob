/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_NEGATIVE_ANSWER_H
#define KNOWROB_NEGATIVE_ANSWER_H

#include "Answer.h"

namespace knowrob {
	/**
	 * A negative answer indicates that a querying component has evidence
	 * for the input query being false for all instances of the query.
	 */
	class AnswerNo : public Answer {
	public:
		/**
		 * Default constructor.
		 */
		AnswerNo();

		/**
		 * Copy constructor.
		 * @param other another answer.
		 */
		AnswerNo(const AnswerNo &other);

		/**
		 * Add a ungroundable literal to the answer.
		 * @param predicate a predicate.
		 * @param isNegated true if the negation of the predicate is ungroundable.
		 */
		void addUngrounded(const std::shared_ptr<Predicate> &predicate, bool isNegated = false);

		/**
		 * Part of the answer is that certain literals that appear positive in the query
		 * are true. This is a list of such literals.
		 * @return a list of positive groundings.
		 */
		auto &positiveUngrounded() const { return positiveUngrounded_; }

		/**
		 * Part of the answer is that certain literals that appear negated in the query
		 * are not true. This is a list of such literals.
		 * @return a list of negative groundings.
		 */
		auto &negativeUngrounded() const { return negativeUngrounded_; }

		// override Token
		size_t hash() const override;

		// override Token
		std::ostream &write(std::ostream &os) const override;

		// override Answer
		std::string toHumanReadableString() const override;

		// override Answer
		bool isNegative() const override { return true; }

		// override Answer
		bool isPositive() const override { return false; }

		// override Answer
		bool isUncertain() const override;

		// override Answer
		void setIsUncertain(std::optional<double> confidence) override;

	protected:
		std::vector<PredicatePtr> positiveUngrounded_;
		std::vector<PredicatePtr> negativeUngrounded_;

		bool isUncertain_;
		std::optional<double> confidence_;
	};

	// alias
	using AnswerNoPtr = std::shared_ptr<const AnswerNo>;

	/**
	 * @return a positive result without additional constraints.
	 */
	const std::shared_ptr<const AnswerNo> &GenericNo();

} // knowrob

#endif //KNOWROB_NEGATIVE_ANSWER_H
