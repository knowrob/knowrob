/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/queries/ConjunctiveBroadcaster.h>
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/terms/Numeric.h"

using namespace knowrob;

ConjunctiveBroadcaster::ConjunctiveBroadcaster(bool ignoreInconsistentAnswers)
		: TokenBroadcaster(),
		  ignoreInconsistentAnswers_(ignoreInconsistentAnswers),
		  hasSolution_(false) {}

void ConjunctiveBroadcaster::push(Channel &channel, const TokenPtr &tok) {
	if (tok->tokenType() == TokenType::ANSWER_TOKEN) {
		auto answer = std::static_pointer_cast<const Answer>(tok);
		if (answer->isPositive()) {
			const uint32_t channelID = channel.id();
			// need to lock the whole push as genCombinations uses an iterator over the buffer.
			std::lock_guard<std::mutex> lock(buffer_mutex_);

			// add to the buffer for later combinations
			// replace other answer with same hash if present
			buffer_[channelID][answer->hashOfAnswer()] = answer;

			// generate combinations with other channels if each channel
			// buffer has some content.
			if (buffer_.size() == channels_.size()) {
				if (channels_.size() == 1) {
					// not needed to generate combinations
					TokenBroadcaster::push(tok);
				} else {
					// generate all combinations and push combined messages
					genCombinations(channelID, buffer_.begin(), answer);
				}
			}
		} else if (answer->isNegative()) {
			// do not combine negative answers like the positive ones above.
			// only push "no" when receiving EOF while no positive answer has been produced.
			negativeAnswers_.emplace_back(std::static_pointer_cast<const AnswerNo>(answer));
		}
	} else {
		if (tok->indicatesEndOfEvaluation()) {
			numClosedChannels_++;
			if (numClosedChannels_ == channels_.size() && !hasSolution_) {
				if (negativeAnswers_.size() == 1) {
					TokenBroadcaster::push(negativeAnswers_.front());
				} else {
					auto no = std::make_shared<AnswerNo>();
					for (auto &x: negativeAnswers_) {
						no->mergeWith(*x);
					}
					TokenBroadcaster::push(no);
				}
			} else {
				// do not pass on the EOF token until all channels have finished
				return;
			}
		}
		// pass through non-answer messages
		TokenStream::push(channel, tok);
	}
}

void ConjunctiveBroadcaster::genCombinations( //NOLINT(misc-no-recursion)
		uint32_t pushedChannelID, AnswerMap::iterator it, AnswerPtr &combinedResult) {
	if (it == buffer_.end()) {
		// end reached, push combination
		TokenBroadcaster::push(combinedResult);
		hasSolution_ = true;
	} else if (it->first == pushedChannelID) {
		// pass through channel from which the new message was pushed
		auto it1 = it;
		++it1;
		genCombinations(pushedChannelID, it1, combinedResult);
	} else if (it->second.size() == 1) {
		// only a single message buffered from this channel
		auto it1 = it;
		++it1;
		auto merged = mergeAnswers(combinedResult,
								   it->second.begin()->second, ignoreInconsistentAnswers_);
		if (merged) {
			genCombinations(pushedChannelID, it1, merged);
		}
	} else {
		// generate a combination for each buffered message
		// note: the number of possible combinations grows exponentially with number of messages in channels
		auto it1 = it;
		++it1;
		for (auto &msg: it->second) {
			auto merged = mergeAnswers(
					combinedResult, msg.second, ignoreInconsistentAnswers_);
			if (merged) {
				genCombinations(pushedChannelID, it1, merged);
			}
		}
	}
}
