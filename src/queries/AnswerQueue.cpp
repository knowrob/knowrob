/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/AnswerQueue.h>

using namespace knowrob;

AnswerQueue::AnswerQueue()
        : AnswerStream()
{}

AnswerQueue::~AnswerQueue()
{
	if(isOpened()) {
		pushToQueue(AnswerStream::eos());
	}
}

void AnswerQueue::pushToQueue(const AnswerPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);
		queue_.push(item);
	}
	queue_CV_.notify_one();
}

void AnswerQueue::push(const AnswerPtr &item)
{
	pushToQueue(item);
}

AnswerPtr& AnswerQueue::front()
{
	std::unique_lock<std::mutex> lock(queue_mutex_);
	queue_CV_.wait(lock, [&]{ return !queue_.empty(); });
	return queue_.front();
}

void AnswerQueue::pop()
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_.pop();
}

AnswerPtr AnswerQueue::pop_front()
{
	AnswerPtr x = front();
	pop();
	return x;
}
