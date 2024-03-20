/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ANSWER_COMBINER_H_
#define KNOWROB_ANSWER_COMBINER_H_

#include <mutex>
#include "TokenBroadcaster.h"
#include "Answer.h"
#include "AnswerNo.h"

namespace knowrob {
	/**
	 * Consolidates answers from multiple sources that are considered in conjunction.
	 * This is intended to be used for parallel evaluation of independent sub-goals within a query.
	 */
	class ConjunctiveBroadcaster : public TokenBroadcaster {
	public:
		explicit ConjunctiveBroadcaster(bool ignoreInconsistentAnswers = true);

	protected:
		// maps channel id to a set of answers in the buffer.
		// a container is used for the answers that removes duplicates.
		using AnswerMap = std::map<uint32_t, std::map<size_t, AnswerPtr>>;

		AnswerMap buffer_;
		std::mutex buffer_mutex_;
		bool ignoreInconsistentAnswers_;
		bool hasSolution_;
		std::vector<AnswerNoPtr> negativeAnswers_;

		// Override AnswerBroadcaster
		void push(Channel &channel, const TokenPtr &tok) override;

		void genCombinations(uint32_t pushedChannelID, AnswerMap::iterator it, AnswerPtr &combinedResult);
	};
}

#endif //KNOWROB_ANSWER_COMBINER_H_
