/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TOKEN_BROADCASTER_H_
#define KNOWROB_TOKEN_BROADCASTER_H_

#include <memory>
#include <list>
#include <knowrob/queries/TokenStream.h>

namespace knowrob {
	/**
	 * A broadcaster of query results.
	 */
	class TokenBroadcaster : public TokenStream {
	public:
		TokenBroadcaster();

		~TokenBroadcaster();

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
		void push(const TokenPtr &tok) override;

		virtual void pushToBroadcast(const TokenPtr &tok);
	};

	void operator>>(const std::shared_ptr<TokenBroadcaster> &a,
					const std::shared_ptr<TokenStream> &b);
}

#endif //KNOWROB_TOKEN_BROADCASTER_H_
