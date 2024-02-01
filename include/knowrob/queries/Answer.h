/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ANSWER_H_
#define KNOWROB_ANSWER_H_

#include "Token.h"
#include "Query.h"

namespace knowrob {
	/**
	 * The answer to a (sub)-query. It can be positive, negative or neither.
	 */
	class Answer : public Token {
	public:
		Answer() = default;

		Answer(const Answer &other) = default;

		/**
		 * @return true if this answer is negative.
		 */
		virtual bool isNegative() const = 0;

		/**
		 * @return true if this answer is positive.
		 */
		virtual bool isPositive() const = 0;

		/**
		 * @return true if truth of this answer is uncertain.
		 */
		virtual bool isUncertain() const = 0;

		/**
		 * @return true if truth of this answer is certain.
		 */
		bool isCertain() const { return !isUncertain(); }

		/**
		 * Mark this answer as uncertain by modification of the associated frame.
		 * @param confidence an optional confidence value.
		 */
		virtual void setIsUncertain(std::optional<double> confidence) = 0;

		/**
		 * @return a human readable string representation of this answer.
		 */
		virtual std::string toHumanReadableString() const = 0;

		// override Token
		bool indicatesEndOfEvaluation() const override { return false; }

		// override Token
		TokenType type() const override { return TokenType::ANSWER_TOKEN; }

		// override Token
		size_t hash() const override;
	};

	// alias
	using AnswerPtr = std::shared_ptr<const Answer>;

	/**
	 * Merge two answers into one.
	 * @param a a answer.
	 * @param b a answer.
	 * @param ignoreInconsistencies if true, inconsistencies are ignored.
	 * @return a merged answer.
	 */
	AnswerPtr mergeAnswers(const AnswerPtr &a, const AnswerPtr &b, bool ignoreInconsistencies);

	/**
	 * Used to compare answers.
	 */
	struct AnswerComparator {
		bool operator()(const AnswerPtr &v0, const AnswerPtr &v1) const;
	};

	/**
	 * A set that removes duplicate answers.
	 */
	using AnswerSet = std::set<AnswerPtr, AnswerComparator>;
}

#endif //KNOWROB_ANSWER_H_
