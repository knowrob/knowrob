/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/queries/AnswerMerger.h"

using namespace knowrob;

AnswerMerger::AnswerMerger(AnswerPtr partialResult)
		: AnswerTransformer(),
		  partialResult_(std::move(partialResult)) {}

TokenPtr AnswerMerger::transform(const TokenPtr &tok) {
	if (tok->type() == TokenType::ANSWER_TOKEN) {
		auto answer = std::static_pointer_cast<const Answer>(tok);
		return mergeAnswers(partialResult_, answer, true);
	} else {
		return tok;
	}
}
