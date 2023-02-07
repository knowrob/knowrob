/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_RESULT_BROADCASTER_H_
#define KNOWROB_QUERY_RESULT_BROADCASTER_H_

#include <memory>
#include <list>
#include <knowrob/queries/QueryResult.h>
#include <knowrob/queries/QueryResultStream.h>

namespace knowrob {
	/**
	 * A broadcaster of query results.
	 */
	class QueryResultBroadcaster : public QueryResultStream {
	public:
		QueryResultBroadcaster();
		~QueryResultBroadcaster();
		
		/**
		 * Add a subscriber to this broadcast.
		 * The subscriber will receive input from the broadcast after this call.
		 * @subscriber a query result stream.
		 */
		void addSubscriber(const std::shared_ptr<Channel> &subscriber);
		
		/**
		 * Remove a previously added subscriber.
		 * @subscriber a query result stream.
		 */
		void removeSubscriber(const std::shared_ptr<Channel> &subscriber);

	protected:
		std::list<std::shared_ptr<Channel>> subscribers_;
		
		// Override QueryResultStream
		void push(const QueryResultPtr &msg) override;
		void pushToBroadcast(const QueryResultPtr &msg);
	};
}

#endif //KNOWROB_QUERY_RESULT_BROADCASTER_H_
