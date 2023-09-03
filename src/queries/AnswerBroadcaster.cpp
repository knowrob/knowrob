/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/queries/AnswerBroadcaster.h>
#include "knowrob/queries/AnswerQueue.h"
#include "knowrob/Logger.h"

using namespace knowrob;

AnswerBroadcaster::AnswerBroadcaster()
: AnswerStream()
{}

AnswerBroadcaster::~AnswerBroadcaster()
{
	if(isOpened()) {
		for(auto &x : subscribers_) {
			x->push(AnswerStream::eos());
		}
	}
}

void AnswerBroadcaster::addSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.push_back(subscriber);
}

void AnswerBroadcaster::removeSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.remove(subscriber);
}

void AnswerBroadcaster::push(const AnswerPtr &item)
{
	pushToBroadcast(item);
}

void AnswerBroadcaster::pushToBroadcast(const AnswerPtr &item)
{
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}

namespace knowrob {
    void operator>>(const std::shared_ptr<AnswerBroadcaster> &a,
                    const std::shared_ptr<AnswerStream> &b)
    {
        a->addSubscriber(AnswerStream::Channel::create(b));
    }
}


// fixture class for testing
class AnswerBroadcasterTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(AnswerBroadcasterTest, OneToOneBroadcast)
{
    auto broadcast = std::make_shared<AnswerBroadcaster>();
    // feed broadcast into queue
    auto output1 = std::make_shared<AnswerQueue>();
    broadcast->addSubscriber(AnswerStream::Channel::create(output1));
    // create channels for broadcast
    auto input1 = AnswerStream::Channel::create(broadcast);
    // push a message
    EXPECT_EQ(output1->size(), 0);
    input1->push(AnswerStream::bos());
    EXPECT_EQ(output1->size(), 1);
    // push more messages
    input1->push(AnswerStream::bos());
    input1->push(AnswerStream::bos());
    EXPECT_EQ(output1->size(), 3);
}

TEST_F(AnswerBroadcasterTest, OneToManyBroadcast)
{
    auto broadcast = std::make_shared<AnswerBroadcaster>();
    // feed broadcast into queue
    const int numSubscriber = 4;
    std::vector<std::shared_ptr<AnswerQueue>> outputs(numSubscriber);
    for(int i=0; i<numSubscriber; ++i) {
        outputs[i] = std::make_shared<AnswerQueue>();
        broadcast->addSubscriber(AnswerStream::Channel::create(outputs[i]));
    }
    // create channels for broadcast
    auto input1 = AnswerStream::Channel::create(broadcast);
    // push a message
    for(int i=0; i<numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 0);
    input1->push(AnswerStream::bos());
    for(int i=0; i<numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 1);
    // push more messages
    input1->push(AnswerStream::bos());
    input1->push(AnswerStream::bos());
    for(int i=0; i<numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 3);
}

TEST_F(AnswerBroadcasterTest, ManyToManyBroadcast)
{
    auto broadcast = std::make_shared<AnswerBroadcaster>();
    // feed broadcast into queue
    const int numSubscriber = 4;
    std::vector<std::shared_ptr<AnswerQueue>> outputs(numSubscriber);
    for(int i=0; i<numSubscriber; ++i) {
        outputs[i] = std::make_shared<AnswerQueue>();
        broadcast->addSubscriber(AnswerStream::Channel::create(outputs[i]));
    }
    // create channels for broadcast
    const int numInputChannels = 3;
    std::vector<std::shared_ptr<AnswerStream::Channel>> inputChannels(numInputChannels);
    for(int i=0; i<numInputChannels; ++i) {
        inputChannels[i] = AnswerStream::Channel::create(broadcast);
    }
    // push a message
    for(int i=0; i<numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 0);
    inputChannels[0]->push(AnswerStream::bos());
    for(int i=0; i<numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 1);
    // push more messages
    inputChannels[0]->push(AnswerStream::bos());
    inputChannels[1]->push(AnswerStream::bos());
    inputChannels[2]->push(AnswerStream::bos());
    for(int i=0; i<numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 4);
}

TEST_F(AnswerBroadcasterTest, MessageAfterEOS)
{
    auto broadcast = std::make_shared<AnswerBroadcaster>();
    // feed broadcast into queue
    auto output1 = std::make_shared<AnswerQueue>();
    broadcast->addSubscriber(AnswerStream::Channel::create(output1));
    // create channels for broadcast
    auto input1 = AnswerStream::Channel::create(broadcast);
    // push a message
    EXPECT_EQ(output1->size(), 0);
    input1->push(AnswerStream::bos());
    EXPECT_EQ(output1->size(), 1);
    // push EOS message
    input1->push(AnswerStream::eos());
    EXPECT_EQ(output1->size(), 2);
    // after EOS, messages will be ignored
    input1->push(AnswerStream::bos());
    EXPECT_EQ(output1->size(), 2);
}

TEST_F(AnswerBroadcasterTest, ManyInputChannelsEOS)
{
    auto broadcast = std::make_shared<AnswerBroadcaster>();
    // feed broadcast into queue
    auto output1 = std::make_shared<AnswerQueue>();
    broadcast->addSubscriber(AnswerStream::Channel::create(output1));
    // create channels for broadcast
    const int numInputChannels = 3;
    std::vector<std::shared_ptr<AnswerStream::Channel>> inputChannels(numInputChannels);
    for(int i=0; i<numInputChannels; ++i) {
        inputChannels[i] = AnswerStream::Channel::create(broadcast);
    }
    // push EOS followed by BOS
    EXPECT_EQ(output1->size(), 0);
    inputChannels[0]->push(AnswerStream::bos());
    inputChannels[0]->push(AnswerStream::eos());
    // EOS is not in queue yet
    EXPECT_EQ(output1->size(), 1);
    inputChannels[1]->push(AnswerStream::bos());
    inputChannels[1]->push(AnswerStream::eos());
    // EOS is not in queue yet
    EXPECT_EQ(output1->size(), 2);
    inputChannels[2]->push(AnswerStream::eos());
    // now EOS is in the queue!
    EXPECT_EQ(output1->size(), 3);
    // stream is closed so inputs will not be processed anymore
    inputChannels[2]->push(AnswerStream::eos());
    EXPECT_EQ(output1->size(), 3);
}
