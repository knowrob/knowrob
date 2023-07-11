/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_RESULT_QUEUE_H_
#define KNOWROB_QUERY_RESULT_QUEUE_H_

#include <queue>
#include <mutex>
#include <condition_variable>
#include <knowrob/queries/AnswerStream.h>

namespace knowrob {
	/**
	 * A queue of QueryResult objects.
	 */
	class AnswerQueue : public AnswerStream {
	public:
		AnswerQueue();

		~AnswerQueue();
		
		/**
		 * Get the front element of this queue without removing it.
		 * This will block until the queue is non empty.
		 * @return the front element of the queue.
		 */
		AnswerPtr& front();
		
		/**
		 * Remove the front element of this queue.
		 */
		void pop();
		
		/**
		 * Get front element and remove it from the queue.
		 * @return the front element of the queue.
		 */
		AnswerPtr pop_front();

        /**
         * @return true if the queue is currently empty.
         */
        bool empty() const { return queue_.empty(); }

	protected:
		std::queue<AnswerPtr> queue_;
		std::condition_variable queue_CV_;
		std::mutex queue_mutex_;

		// Override QueryResultStream
		void push(const AnswerPtr &item) override;
		void pushToQueue(const AnswerPtr &item);
	};
}

#endif //KNOWROB_QUERY_RESULT_QUEUE_H_
