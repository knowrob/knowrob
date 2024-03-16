/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/queries/TokenBroadcaster.h>
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/Logger.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

TokenBroadcaster::TokenBroadcaster()
		: TokenStream() {}

TokenBroadcaster::~TokenBroadcaster() {
	if (isOpened()) {
		for (auto &x: subscribers_) {
			x->push(EndOfEvaluation::get());
		}
	}
}

void TokenBroadcaster::addSubscriber(const std::shared_ptr<Channel> &subscriber) {
	subscribers_.push_back(subscriber);
}

void TokenBroadcaster::removeSubscriber(const std::shared_ptr<Channel> &subscriber) {
	subscribers_.remove(subscriber);
}

void TokenBroadcaster::push(const TokenPtr &tok) {
	pushToBroadcast(tok);
}

void TokenBroadcaster::pushToBroadcast(const TokenPtr &tok) {
	// broadcast the query result to all subscribers
	for (auto &x: subscribers_) {
		x->push(tok);
	}
}

namespace knowrob {
	void operator>>(const std::shared_ptr<TokenBroadcaster> &a,
					const std::shared_ptr<TokenStream> &b) {
		a->addSubscriber(TokenStream::Channel::create(b));
	}
}

namespace knowrob::py {
	template<>
	void createType<TokenBroadcaster>() {
		using namespace boost::python;
		class_<TokenBroadcaster, std::shared_ptr<TokenBroadcaster>, bases<TokenStream>, boost::noncopyable>
				("TokenBroadcaster", init<>())
				.def("addSubscriber", &TokenBroadcaster::addSubscriber)
				.def("removeSubscriber", &TokenBroadcaster::removeSubscriber);
	}
}

// fixture class for testing
namespace knowrob::testing {
	class AnswerBroadcasterTest : public ::testing::Test {
	protected:
		void SetUp() override {}

		void TearDown() override {}
	};
}
using namespace knowrob::testing;

TEST_F(AnswerBroadcasterTest, OneToOneBroadcast) {
	auto broadcast = std::make_shared<TokenBroadcaster>();
	// feed broadcast into queue
	auto output1 = std::make_shared<TokenQueue>();
	broadcast->addSubscriber(TokenStream::Channel::create(output1));
	// create channels for broadcast
	auto input1 = TokenStream::Channel::create(broadcast);
	// push a message
	EXPECT_EQ(output1->size(), 0);
	input1->push(GenericYes());
	EXPECT_EQ(output1->size(), 1);
	// push more messages
	input1->push(GenericYes());
	input1->push(GenericYes());
	EXPECT_EQ(output1->size(), 3);
}

TEST_F(AnswerBroadcasterTest, OneToManyBroadcast) {
	auto broadcast = std::make_shared<TokenBroadcaster>();
	// feed broadcast into queue
	const int numSubscriber = 4;
	std::vector<std::shared_ptr<TokenQueue>> outputs(numSubscriber);
	for (int i = 0; i < numSubscriber; ++i) {
		outputs[i] = std::make_shared<TokenQueue>();
		broadcast->addSubscriber(TokenStream::Channel::create(outputs[i]));
	}
	// create channels for broadcast
	auto input1 = TokenStream::Channel::create(broadcast);
	// push a message
	for (int i = 0; i < numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 0);
	input1->push(GenericYes());
	for (int i = 0; i < numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 1);
	// push more messages
	input1->push(GenericYes());
	input1->push(GenericYes());
	for (int i = 0; i < numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 3);
}

TEST_F(AnswerBroadcasterTest, ManyToManyBroadcast) {
	auto broadcast = std::make_shared<TokenBroadcaster>();
	// feed broadcast into queue
	const int numSubscriber = 4;
	std::vector<std::shared_ptr<TokenQueue>> outputs(numSubscriber);
	for (int i = 0; i < numSubscriber; ++i) {
		outputs[i] = std::make_shared<TokenQueue>();
		broadcast->addSubscriber(TokenStream::Channel::create(outputs[i]));
	}
	// create channels for broadcast
	const int numInputChannels = 3;
	std::vector<std::shared_ptr<TokenStream::Channel>> inputChannels(numInputChannels);
	for (int i = 0; i < numInputChannels; ++i) {
		inputChannels[i] = TokenStream::Channel::create(broadcast);
	}
	// push a message
	for (int i = 0; i < numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 0);
	inputChannels[0]->push(GenericYes());
	for (int i = 0; i < numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 1);
	// push more messages
	inputChannels[0]->push(GenericYes());
	inputChannels[1]->push(GenericYes());
	inputChannels[2]->push(GenericYes());
	for (int i = 0; i < numSubscriber; ++i) EXPECT_EQ(outputs[i]->size(), 4);
}

TEST_F(AnswerBroadcasterTest, MessageAfterEOS) {
	auto broadcast = std::make_shared<TokenBroadcaster>();
	// feed broadcast into queue
	auto output1 = std::make_shared<TokenQueue>();
	broadcast->addSubscriber(TokenStream::Channel::create(output1));
	// create channels for broadcast
	auto input1 = TokenStream::Channel::create(broadcast);
	// push a message
	EXPECT_EQ(output1->size(), 0);
	input1->push(GenericYes());
	EXPECT_EQ(output1->size(), 1);
	// push EOS message
	input1->push(EndOfEvaluation::get());
	EXPECT_EQ(output1->size(), 2);
	// after EOS, messages will be ignored
	input1->push(GenericYes());
	EXPECT_EQ(output1->size(), 2);
}

TEST_F(AnswerBroadcasterTest, ManyInputChannelsEOS) {
	auto broadcast = std::make_shared<TokenBroadcaster>();
	// feed broadcast into queue
	auto output1 = std::make_shared<TokenQueue>();
	broadcast->addSubscriber(TokenStream::Channel::create(output1));
	// create channels for broadcast
	const int numInputChannels = 3;
	std::vector<std::shared_ptr<TokenStream::Channel>> inputChannels(numInputChannels);
	for (int i = 0; i < numInputChannels; ++i) {
		inputChannels[i] = TokenStream::Channel::create(broadcast);
	}
	// push EOS followed by BOS
	EXPECT_EQ(output1->size(), 0);
	inputChannels[0]->push(GenericYes());
	inputChannels[0]->push(EndOfEvaluation::get());
	// EOS is not in queue yet
	EXPECT_EQ(output1->size(), 1);
	inputChannels[1]->push(GenericYes());
	inputChannels[1]->push(EndOfEvaluation::get());
	// EOS is not in queue yet
	EXPECT_EQ(output1->size(), 2);
	inputChannels[2]->push(EndOfEvaluation::get());
	// now EOS is in the queue!
	EXPECT_EQ(output1->size(), 3);
	// stream is closed so inputs will not be processed anymore
	inputChannels[2]->push(EndOfEvaluation::get());
	EXPECT_EQ(output1->size(), 3);
}
