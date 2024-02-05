//
// Created by daniel on 30.01.24.
//

#ifndef KNOWROB_ANSWER_CONSOLIDATOR_H
#define KNOWROB_ANSWER_CONSOLIDATOR_H

#include "TokenBroadcaster.h"
#include "AnswerYes.h"
#include "AnswerNo.h"
#include "AnswerDontKnow.h"

namespace knowrob {
	/**
	 * Consolidates answers from multiple sources that are considered in disjunction.
	 */
	class DisjunctiveBroadcaster : public TokenBroadcaster {
	public:
		DisjunctiveBroadcaster();

	protected:
		std::vector<AnswerNoPtr> negativeAnswers_;
		std::vector<AnswerYesPtr> deferredPositiveAnswers_;
		bool isCertainlyPositive_;

		// Override AnswerStream
		void push(const TokenPtr &msg) override;

		void pushAnswer(const AnswerPtr &answer);

		void pushDeferredMessages();
	};

} // knowrob

#endif //KNOWROB_ANSWER_CONSOLIDATOR_H
