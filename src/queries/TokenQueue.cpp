/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/TokenQueue.h>
#include "knowrob/py/utils.h"

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

namespace knowrob::py {
	template<>
	void createType<TokenQueue>() {
		using namespace boost::python;
		class_<TokenQueue, std::shared_ptr<TokenQueue>, bases<TokenStream>, boost::noncopyable>
				("TokenQueue", init<>())
				.def("front", &TokenQueue::front, return_value_policy<reference_existing_object>())
				.def("pop", &TokenQueue::pop)
				.def("pop_front", &TokenQueue::pop_front)
				.def("empty", &TokenQueue::empty)
				.def("size", &TokenQueue::size);
	}
}
