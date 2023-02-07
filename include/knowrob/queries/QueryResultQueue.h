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
#include <knowrob/queries/QueryResultStream.h>

namespace knowrob {
	/**
	 * A queue of QueryResult objects.
	 */
	class QueryResultQueue : public QueryResultStream {
	public:
		QueryResultQueue();
		~QueryResultQueue();
		
		/**
		 * Get the front element of this queue without removing it.
		 * This will block until the queue is non empty.
		 * @return the front element of the queue.
		 */
		QueryResultPtr& front();
		
		/**
		 * Remove the front element of this queue.
		 */
		void pop();
		
		/**
		 * Get front element and remove it from the queue.
		 * @return the front element of the queue.
		 */
		QueryResultPtr pop_front();

	protected:
		std::queue<QueryResultPtr> queue_;
		std::condition_variable queue_CV_;
		std::mutex queue_mutex_;
		
		// Override QueryResultStream
		void push(const QueryResultPtr &item) override;
		void pushToQueue(const QueryResultPtr &item);
	};
}

#endif //KNOWROB_QUERY_RESULT_QUEUE_H_
