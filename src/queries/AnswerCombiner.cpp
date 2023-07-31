/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/queries/AnswerCombiner.h>
#include "knowrob/Logger.h"
#include "knowrob/queries/AnswerQueue.h"
#include "knowrob/queries/QueryParser.h"

using namespace knowrob;

AnswerCombiner::AnswerCombiner()
: AnswerBroadcaster()
{}

void AnswerCombiner::push(const Channel &channel, const AnswerPtr &msg)
{
	const uint32_t channelID = channel.id();
	// need to lock the whole push as genCombinations uses an iterator over the buffer.
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	
	// add to the buffer for later combinations
	buffer_[channelID].push_back(msg);

	// generate combinations with other channels if each channel
	// buffer has some content.
	if(buffer_.size() == channels_.size()) {
        if(channels_.size()==1) {
            // not needed to generate combinations
            AnswerBroadcaster::push(msg);
        }
        else {
            std::shared_ptr<Answer> combination(new Answer(*(msg.get())));
            // generate all combinations and push combined messages
            genCombinations(channelID, buffer_.begin(), combination);
        }
	}
}

void AnswerCombiner::genCombinations( //NOLINT
		uint32_t pushedChannelID,
        AnswerMap::iterator it,
        std::shared_ptr<Answer> &combinedResult)
{
	if(it == buffer_.end()) {
		// end reached, push combination
		// note: need to create a new SubstitutionPtr due to the rollBack below.
		AnswerBroadcaster::push(std::make_shared<Answer>(*combinedResult));
	}
	else if(it->first == pushedChannelID) {
		// pass through channel from which the new message was pushed
		auto it1 = it; ++it1;
		genCombinations(pushedChannelID, it1, combinedResult);
	}
	else if(it->second.size()==1) {
		// only a single message buffered from this channel
		auto it1 = it; ++it1;
		// keep track of changes for rolling them back
		Reversible changes;
		// combine and continue with next channel.
		// this is done to avoid creating many copies of the substitution map.
		if(combinedResult->combine(it->second.front(), &changes)) {
			genCombinations(pushedChannelID, it1, combinedResult);
		}
		// roll back changes
		changes.rollBack();
	}
	else {
		// generate a combination for each buffered message
		// note: the number of possible combinations grows
		// exponentially with number of messages in channels
		// TODO: it might be good to think about ways of discarding options quicker.
		// - one way would be to cache all possible combinations, but could be many of them.
		//   then only these would need to be iterated here, but additionally pre-computing
		//   must happen here
		auto it1 = it; ++it1;
		// keep track of changes for rolling them back
		Reversible changes;
		// combine each buffered message
		for(auto &msg : it->second) {
			// combine and continue with next channel
			if(combinedResult->combine(msg, &changes)) {
				genCombinations(pushedChannelID, it1, combinedResult);
			}
			// roll back changes, note this also clears the `changes` object
			// so it can be re-used in the next iteration
			changes.rollBack();
		}
	}
}


// fixture class for testing
class AnswerCombinerTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(AnswerCombinerTest, CombineOne)
{
    auto combiner = std::make_shared<AnswerCombiner>();
    // feed broadcast into queue
    auto output = std::make_shared<AnswerQueue>();
    combiner->addSubscriber(AnswerStream::Channel::create(output));
    // create channels for broadcast
    auto input1 = AnswerStream::Channel::create(combiner);
    // push a message
    EXPECT_EQ(output->size(), 0);
    input1->push(AnswerStream::bos());
    EXPECT_EQ(output->size(), 1);
    // push more messages
    input1->push(AnswerStream::bos());
    input1->push(AnswerStream::bos());
    EXPECT_EQ(output->size(), 3);
}

TEST_F(AnswerCombinerTest, CombineMany_DifferentVariables)
{
    auto combiner = std::make_shared<AnswerCombiner>();
    // feed broadcast into queue
    auto output = std::make_shared<AnswerQueue>();
    combiner->addSubscriber(AnswerStream::Channel::create(output));
    // create channels for broadcast
    auto input1 = AnswerStream::Channel::create(combiner);
    auto input2 = AnswerStream::Channel::create(combiner);
    // construct partial answers
    auto answer11 = std::make_shared<Answer>();
    auto answer21 = std::make_shared<Answer>();
    auto answer22 = std::make_shared<Answer>();
    answer11->substitute(Variable("a"), std::make_shared<Integer32Term>(4));
    answer21->substitute(Variable("b"), std::make_shared<Integer32Term>(6));
    answer22->substitute(Variable("b"), std::make_shared<Integer32Term>(7));
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

TEST_F(AnswerCombinerTest, CombineMany_Unification)
{
    auto combiner = std::make_shared<AnswerCombiner>();
    // feed broadcast into queue
    auto output = std::make_shared<AnswerQueue>();
    combiner->addSubscriber(AnswerStream::Channel::create(output));
    // create channels for broadcast
    auto input1 = AnswerStream::Channel::create(combiner);
    auto input2 = AnswerStream::Channel::create(combiner);
    // construct partial answers
    auto answer11 = std::make_shared<Answer>();
    auto answer21 = std::make_shared<Answer>();
    auto answer22 = std::make_shared<Answer>();
    // push "a=p(X,1)" and "a=p(2,Y)" into combiner via two channels
    answer11->substitute(Variable("a"), QueryParser::parsePredicate("p(X,1)"));
    answer21->substitute(Variable("a"), QueryParser::parsePredicate("p(2,Y)"));
    answer22->substitute(Variable("a"), QueryParser::parsePredicate("p(2,2)"));
    input1->push(answer11);
    input2->push(answer21);
    // expect that the combiner has one output "a=p(2,1)" unifying both inputs.
    EXPECT_EQ(output->size(), 1);
    if(output->size()==1) {
        auto combinedResult = output->front();
        EXPECT_TRUE(combinedResult->hasSubstitution(Variable("a")));
        auto instantiation = combinedResult->substitution()->get(Variable("a"));
        EXPECT_EQ(*instantiation, *QueryParser::parsePredicate("p(2,1)"));
    }
    // "a=p(X,1)" and "a=p(2,2)" cannot be combined, number of outputs stays at 1
    input2->push(answer22);
    EXPECT_EQ(output->size(), 1);
}
