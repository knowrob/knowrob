/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/QueryResultQueue.h>

using namespace knowrob;

QueryResultQueue::QueryResultQueue()
: QueryResultStream()
{}

QueryResultQueue::~QueryResultQueue()
{
	if(isOpened()) {
		pushToQueue(QueryResultStream::eos());
	}
}

void QueryResultQueue::pushToQueue(const QueryResultPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);
		queue_.push(item);
	}
	queue_CV_.notify_one();
}

void QueryResultQueue::push(const QueryResultPtr &item)
{
	pushToQueue(item);
}

QueryResultPtr& QueryResultQueue::front()
{
	std::unique_lock<std::mutex> lock(queue_mutex_);
	queue_CV_.wait(lock, [&]{ return !queue_.empty(); });
	return queue_.front();
}

void QueryResultQueue::pop()
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_.pop();
}

QueryResultPtr QueryResultQueue::pop_front()
{
	QueryResultPtr x = front();
	pop();
	return x;
}
