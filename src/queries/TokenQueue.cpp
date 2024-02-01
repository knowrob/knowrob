/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/TokenQueue.h>

using namespace knowrob;

TokenQueue::TokenQueue()
		: TokenStream() {}

TokenQueue::~TokenQueue() {
	if (isOpened()) {
		pushToQueue(EndOfEvaluation::get());
	}
}

void TokenQueue::pushToQueue(const TokenPtr &tok) {
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);
		queue_.push(tok);
	}
	queue_CV_.notify_one();
}

void TokenQueue::push(const TokenPtr &tok) {
	pushToQueue(tok);
}

TokenPtr &TokenQueue::front() {
	std::unique_lock<std::mutex> lock(queue_mutex_);
	queue_CV_.wait(lock, [&] { return !queue_.empty(); });
	return queue_.front();
}

void TokenQueue::pop() {
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_.pop();
}

TokenPtr TokenQueue::pop_front() {
	TokenPtr x = front();
	pop();
	return x;
}
