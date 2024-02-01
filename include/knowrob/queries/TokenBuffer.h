/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TOKEN_BUFFER_H
#define KNOWROB_TOKEN_BUFFER_H

#include "TokenBroadcaster.h"
#include "TokenQueue.h"

namespace knowrob {
	/**
	 * A token stream that buffers all input until stopBuffering() is called.
	 * The main purpose is to avoid losing messages while building a pipeline,
	 * such that pipeline can be partially active during construction.
	 * Also TokenBuffer provides an easy exit point for a pipeline by creating a queue.
	 */
	class TokenBuffer : public TokenBroadcaster {
	public:
		TokenBuffer();

		/**
		 * Stop buffering and forward all buffered tokens to subscribers.
		 */
		void stopBuffering();

		/**
		 * Create a queue that will receive all output tokens of this stream.
		 * @return a queue of tokens.
		 */
		std::shared_ptr<TokenQueue> createQueue();

	protected:
		std::atomic<bool> isBuffering_;
		std::list<TokenPtr> buffer_;

		// Override QueryResultStream
		void push(const TokenPtr &tok) override;
	};

	using TokenBufferPtr = std::shared_ptr<TokenBuffer>;

} // knowrob

#endif //KNOWROB_TOKEN_BUFFER_H
