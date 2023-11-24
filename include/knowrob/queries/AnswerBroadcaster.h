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
#include <knowrob/queries/Answer.h>
#include <knowrob/queries/AnswerStream.h>

namespace knowrob {
	/**
	 * A broadcaster of query results.
	 */
	class AnswerBroadcaster : public AnswerStream {
	public:
		AnswerBroadcaster();
		~AnswerBroadcaster();
		
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
		void push(const AnswerPtr &msg) override;
		virtual void pushToBroadcast(const AnswerPtr &msg);
	};

    void operator>>(const std::shared_ptr<AnswerBroadcaster> &a,
                    const std::shared_ptr<AnswerStream> &b);
}

#endif //KNOWROB_QUERY_RESULT_BROADCASTER_H_
