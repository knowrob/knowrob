/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TOKEN_QUEUE_H_
#define KNOWROB_TOKEN_QUEUE_H_

#include <queue>
#include <mutex>
#include <condition_variable>
#include <knowrob/queries/TokenStream.h>

namespace knowrob {
	/**
	 * A queue of QueryResult objects.
	 */
	class TokenQueue : public TokenStream {
	public:
		TokenQueue();

		~TokenQueue();

		/**
		 * Get the front element of this queue without removing it.
		 * This will block until the queue is non empty.
		 * @return the front element of the queue.
		 */
		TokenPtr &front();

		/**
		 * Remove the front element of this queue.
		 */
		void pop();

		/**
		 * Get front element and remove it from the queue.
		 * @return the front element of the queue.
		 */
		TokenPtr pop_front();

		/**
		 * @return true if the queue is currently empty.
		 */
		bool empty() const { return queue_.empty(); }

		/**
		 * @return number of currently queued elements.
		 */
		auto size() const { return queue_.size(); }

	protected:
		std::queue<TokenPtr> queue_;
		std::condition_variable queue_CV_;
		std::mutex queue_mutex_;

		// Override QueryResultStream
		void push(const TokenPtr &tok) override;

		void pushToQueue(const TokenPtr &tok);
	};
}

#endif //KNOWROB_TOKEN_QUEUE_H_
