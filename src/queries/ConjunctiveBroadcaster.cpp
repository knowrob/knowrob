/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/queries/ConjunctiveBroadcaster.h>
#include "knowrob/Logger.h"
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/terms/Numeric.h"

using namespace knowrob;

ConjunctiveBroadcaster::ConjunctiveBroadcaster(bool ignoreInconsistentAnswers)
		: TokenBroadcaster(),
		  ignoreInconsistentAnswers_(ignoreInconsistentAnswers),
		  hasSolution_(false) {}

void ConjunctiveBroadcaster::push(Channel &channel, const TokenPtr &tok) {
	if (tok->type() == TokenType::ANSWER_TOKEN) {
		auto answer = std::static_pointer_cast<const Answer>(tok);
		if (answer->isPositive()) {
			const uint32_t channelID = channel.id();
			// need to lock the whole push as genCombinations uses an iterator over the buffer.
			std::lock_guard<std::mutex> lock(buffer_mutex_);

			// add to the buffer for later combinations
			// replace other answer with same hash if present
			buffer_[channelID][answer->hash()] = answer;

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
		if (tok->indicatesEndOfEvaluation() && !hasSolution_) {
			if(negativeAnswers_.size()==1) {
				TokenBroadcaster::push(negativeAnswers_.front());
			}
			else {
				auto no = std::make_shared<AnswerNo>();
				for (auto &x: negativeAnswers_) {
					no->mergeWith(*x);
				}
				TokenBroadcaster::push(no);
			}
		}
		// pass through non-answer messages
		TokenStream::push(channel, tok);
	}
}

void ConjunctiveBroadcaster::genCombinations( //NOLINT
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


// fixture class for testing
namespace knowrob::testing {
	class AnswerCombinerTest : public ::testing::Test {
	protected:
		void SetUp() override {}

		void TearDown() override {}
	};
}
using namespace knowrob::testing;

TEST_F(AnswerCombinerTest, CombineOne) {
	auto combiner = std::make_shared<ConjunctiveBroadcaster>();
	// feed broadcast into queue
	auto output = std::make_shared<TokenQueue>();
	combiner->addSubscriber(TokenStream::Channel::create(output));
	// create channels for broadcast
	auto input1 = TokenStream::Channel::create(combiner);
	// push a message
	EXPECT_EQ(output->size(), 0);
	input1->push(GenericYes());
	EXPECT_EQ(output->size(), 1);
	// push more messages
	input1->push(GenericYes());
	input1->push(GenericYes());
	EXPECT_EQ(output->size(), 3);
}

TEST_F(AnswerCombinerTest, CombineMany_DifferentVariables) {
	auto combiner = std::make_shared<ConjunctiveBroadcaster>();
	// feed broadcast into queue
	auto output = std::make_shared<TokenQueue>();
	combiner->addSubscriber(TokenStream::Channel::create(output));
	// create channels for broadcast
	auto input1 = TokenStream::Channel::create(combiner);
	auto input2 = TokenStream::Channel::create(combiner);
	// construct partial answers
	auto answer11 = std::make_shared<AnswerYes>(std::make_shared<Bindings>(Bindings::VarMap{
		{std::make_shared<Variable>("a"), std::make_shared<Integer>(4)}
	}));
	auto answer21 = std::make_shared<AnswerYes>(std::make_shared<Bindings>(Bindings::VarMap{
		{std::make_shared<Variable>("b"), std::make_shared<Integer>(6)}
	}));
	auto answer22 = std::make_shared<AnswerYes>(std::make_shared<Bindings>(Bindings::VarMap{
		{std::make_shared<Variable>("b"), std::make_shared<Integer>(7)}
	}));
	// push a partial answer via input1
	EXPECT_EQ(output->size(), 0);
	input1->push(answer11);
	EXPECT_EQ(output->size(), 0);
	// push a partial answer via input2
	input2->push(answer21);
	EXPECT_EQ(output->size(), 1);
	input2->push(answer22);
	EXPECT_EQ(output->size(), 2);
}

TEST_F(AnswerCombinerTest, CombineMany_Unification) {
	auto combiner = std::make_shared<ConjunctiveBroadcaster>(false);
	// feed broadcast into queue
	auto output = std::make_shared<TokenQueue>();
	combiner->addSubscriber(TokenStream::Channel::create(output));
	// create channels for broadcast
	auto input1 = TokenStream::Channel::create(combiner);
	auto input2 = TokenStream::Channel::create(combiner);
	// construct partial answers
	auto answer11 = std::make_shared<AnswerYes>(std::make_shared<Bindings>(Bindings::VarMap{
		{std::make_shared<Variable>("a"), QueryParser::parseFunction("p(X,1)")}
	}));
	auto answer21 = std::make_shared<AnswerYes>(std::make_shared<Bindings>(Bindings::VarMap{
		{std::make_shared<Variable>("a"), QueryParser::parseFunction("p(2,Y)")}
	}));
	auto answer22 = std::make_shared<AnswerYes>(std::make_shared<Bindings>(Bindings::VarMap{
		{std::make_shared<Variable>("a"), QueryParser::parseFunction("p(2,2)")}
	}));
	// push "a=p(X,1)" and "a=p(2,Y)" into combiner via two channels
	input1->push(answer11);
	input2->push(answer21);
	// expect that the combiner has one output "a=p(2,1)" unifying both inputs.
	EXPECT_EQ(output->size(), 1);
	if (output->size() == 1) {
		auto combinedResult = output->front();
		EXPECT_EQ(combinedResult->type(), TokenType::ANSWER_TOKEN);
		if (combinedResult->type() == TokenType::ANSWER_TOKEN) {
			auto answer = std::static_pointer_cast<const Answer>(combinedResult);
			EXPECT_TRUE(answer->isPositive());
			EXPECT_FALSE(answer->isNegative());
			EXPECT_FALSE(answer->isUncertain());
			if (answer->isPositive()) {
				auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);
				EXPECT_TRUE(positiveAnswer->hasGrounding(Variable("a")));
				auto instantiation = positiveAnswer->substitution()->get("a");
				EXPECT_EQ(*instantiation, *QueryParser::parseFunction("p(2,1)"));
			}
		}
	}
	// "a=p(X,1)" and "a=p(2,2)" cannot be combined, number of outputs stays at 1
	input2->push(answer22);
	EXPECT_EQ(output->size(), 1);
}
