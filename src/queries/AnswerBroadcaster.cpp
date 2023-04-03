/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/AnswerBroadcaster.h>

using namespace knowrob;

AnswerBroadcaster::AnswerBroadcaster()
: AnswerStream()
{}

AnswerBroadcaster::~AnswerBroadcaster()
{
	if(isOpened()) {
		pushToBroadcast(AnswerStream::eos());
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
