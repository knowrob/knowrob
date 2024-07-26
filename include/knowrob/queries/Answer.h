/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ANSWER_H_
#define KNOWROB_ANSWER_H_

#include "Token.h"
#include "Query.h"
#include "knowrob/terms/Atom.h"

namespace knowrob {
	/**
	 * The answer to a (sub)-query. It can be positive, negative or neither.
	 */
	class Answer : public Token {
	public:
		Answer()
				: Token(TokenType::ANSWER_TOKEN),
				  frame_(std::make_shared<GraphSelector>()) {}

		Answer(const Answer &other)
				: Token(TokenType::ANSWER_TOKEN),
				  frame_(other.frame_),
				  reasonerTerm_(other.reasonerTerm_) {};

		virtual ~Answer() = default;

		/**
		 * The answer is framed in the context of a graph selector which determines
		 * the set of graphs in which the answer is valid.
		 * This can be used to e.g. address graphs that represent the world state from the
		 * perspective of a specific agent, or a specific point in time.
		 * @return a graph selector.
		 */
		auto &frame() const { return frame_; }

		/**
		 * Assign a graph selector to this answer.
		 * @param frame a graph selector.
		 */
		void setFrame(const std::shared_ptr<GraphSelector> &frame);

		/**
		 * Apply a frame to this answer.
		 * @param frame a graph selector.
		 */
		void applyFrame(const GraphSelector &frame);

		/**
		 * @return true if this answer is negative.
		 */
		bool isNegative() const { return isNegative_; }

		/**
		 * @return true if this answer is positive.
		 */
		bool isPositive() const { return isPositive_; }

		/**
		 * @return true if truth of this answer is uncertain.
		 */
		bool isUncertain() const;

		/**
		 * @return true if truth of this answer is certain.
		 */
		bool isCertain() const { return !isUncertain(); }

		/**
		 * Mark this answer as uncertain by modification of the associated frame.
		 * @param val true if the answer is uncertain.
		 * @param confidence the confidence value.
		 */
		void setIsUncertain(bool val, std::optional<double> confidence);

		/**
		 * @return true if truth of this answer is uncertain.
		 */
		bool isOccasionallyTrue() const;

		/**
		 * @return true if truth of this answer is certain.
		 */
		bool isAllwaysTrue() const { return !isOccasionallyTrue(); }

		/**
		 * Mark this answer as occasionally true.
		 * @param val true if the answer is occasionally true.
		 */
		void setIsOccasionallyTrue(bool val);

		/**
		 * @return the name of the reasoner that was used to generate this answer.
		 */
		void setReasonerTerm(const AtomPtr &reasonerTerm) { reasonerTerm_ = reasonerTerm; }

		/**
		 * @return the name of the reasoner that was used to generate this answer.
		 */
		auto &reasonerTerm() const { return reasonerTerm_; }

		/**
		 * @return the hash of this answer.
		 */
		size_t hashOfAnswer() const;

		/**
		 * @return a string representation of this answer.
		 */
		std::string stringFormOfAnswer() const;

		/**
		 * @return a human readable string representation of this answer.
		 */
		std::string humanReadableForm() const;

	protected:
		std::shared_ptr<GraphSelector> frame_;
		AtomPtr reasonerTerm_;
		bool isPositive_ = false;
		bool isNegative_ = false;

		void setIsPositive(bool val) { isPositive_ = val; }

		void setIsNegative(bool val) { isNegative_ = val; }
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
	using AnswerHandler = std::function<void(const AnswerPtr &)>;
}

#endif //KNOWROB_ANSWER_H_
