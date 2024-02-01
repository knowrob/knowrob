/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/DisjunctiveBroadcaster.h"

using namespace knowrob;

DisjunctiveBroadcaster::DisjunctiveBroadcaster()
		: TokenBroadcaster(),
		  isCertainlyPositive_(false) {
}

void DisjunctiveBroadcaster::pushDeferredMessages() {
	if (isCertainlyPositive_) {
		for (auto &x: deferredPositiveAnswers_) {
			TokenBroadcaster::push(x);
		}
	} else {
		// neither a certain positive nor a certain negative answer has been produced.
		if (deferredPositiveAnswers_.empty()) {
			// TODO: merge all negative answers into one?
			if (!negativeAnswers_.empty()) {
				TokenBroadcaster::push(negativeAnswers_.back());
			}
		} else {
			// only push positive answers if there are any
			// TODO: merge all positive answers into one?
			for (auto &x: deferredPositiveAnswers_) {
				TokenBroadcaster::push(x);
			}
		}
	}
	deferredPositiveAnswers_.clear();
	negativeAnswers_.clear();
}

void DisjunctiveBroadcaster::pushAnswer(const AnswerPtr &answer) {
	// note that this stage consolidates results of independent evaluations that
	// are considered in disjunction. Thus, a positive answer from one evaluation
	// is sufficient to consider the query as satisfiable.
	// However, a negative answer from one evaluation is not sufficient to consider
	// the query as unsatisfiable, as there might be other evaluations that
	// consider the query as satisfiable.

	if (answer->isNegative()) {
		negativeAnswers_.emplace_back(std::static_pointer_cast<const AnswerNo>(answer));
	} else if (answer->isPositive()) {
		// a positive answer indicates that a subsystem suggests the input query is satisfiable.
		// however, the answer can be uncertain, so the "YES" answer is only
		// pushed directly in case it is a certain answer.
		// pushing of uncertain answers is delayed until a certain answer has been produced,
		// or until the end of the stream is reached.
		auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);

		if (positiveAnswer->isCertain()) {
			isCertainlyPositive_ = true;
			TokenBroadcaster::push(answer);
			pushDeferredMessages();
		} else if (isCertainlyPositive_) {
			// push uncertain positive message if a certain positive answer has been produced
			TokenBroadcaster::push(answer);
		} else {
			// else defer pushing uncertain positive answer until a certain answer has been produced, or eof reached
			deferredPositiveAnswers_.emplace_back(positiveAnswer);
		}
	} else {
		// neither a positive nor a negative answer, i.e. reasoning system has no clue
	}
}

void DisjunctiveBroadcaster::push(const TokenPtr &tok) {
	if (tok->indicatesEndOfEvaluation()) {
		// end of stream message indicates that the input stream has been closed.
		pushDeferredMessages();
		TokenBroadcaster::push(tok);
	} else if (tok->type() == TokenType::ANSWER_TOKEN) {
		pushAnswer(std::static_pointer_cast<const Answer>(tok));
	} else {
		TokenBroadcaster::push(tok);
	}
}
