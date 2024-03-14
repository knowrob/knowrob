/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ANSWER_DONT_KNOW_H
#define KNOWROB_ANSWER_DONT_KNOW_H

#include "Answer.h"

namespace knowrob {
	/**
	 * The answer is neither "yes" nor "no", but rather "I don't know".
	 */
	class AnswerDontKnow : public Answer {
	public:
		AnswerDontKnow();

		AnswerDontKnow(const AnswerDontKnow &other);

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
	};

	// alias
	using AnswerDontKnowPtr = std::shared_ptr<const AnswerDontKnow>;

	/**
	 * @return a positive result without additional constraints.
	 */
	const std::shared_ptr<const AnswerDontKnow> &GenericDontKnow();

} // knowrob

#endif //KNOWROB_ANSWER_DONT_KNOW_H
