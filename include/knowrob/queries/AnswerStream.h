/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_RESULT_STREAM_H_
#define KNOWROB_QUERY_RESULT_STREAM_H_

#include <memory>
#include <mutex>
#include <atomic>
#include <list>
#include <knowrob/queries/Answer.h>

namespace knowrob {
	/**
	 * A stream of query results.
	 * The only way to write to a stream is by creating a channel.
	 */
	class AnswerStream {
	public:
		AnswerStream();
		~AnswerStream();
		AnswerStream(const AnswerStream&) = delete;
		
		/**
		 * Find out if a message indicates the end-of-stream (EOS).
		 * @msg a QueryResult pointer.
		 * @return true if the result indicates EOS.
		 */
		static bool isEOS(const AnswerPtr &msg);
		
		/**
		 * Get the end-of-stream (eos) message.
		 * @return the eos message.
		 */
		static AnswerPtr& eos();
		
		/**
		 * Get the begin-of-stream (bos) message.
		 * This is basically an empty substitution mapping.
		 * @return the bos message.
		 */
		static AnswerPtr& bos();
		
		/**
		 * Close the stream.
		 * This will push an EOS message, and all future
		 * attempts to push a non EOS message will cause an error.
		 * Once closed, a stream cannot be opened again.
		 * Note that a stream auto-closes once it has received EOS
		 * messages from all of its input channels.
		 */
		virtual void close();
		
		/**
		 * @return true if opened.
		 */
		bool isOpened() const;

		/**
		 * An input channel of a stream.
		 */
		class Channel {
		public:
			/**
			 * @param stream the query result stream associated to this channel.
			 */
			explicit Channel(const std::shared_ptr<AnswerStream> &stream);

			~Channel();

			/**
			 * Cannot be copy-assigned.
			 */
			Channel(const Channel&) = delete;

			/**
			 * Create a new stream channel.
			 * Note that this will generate an error in case the stream
			 * is closed already.
			 * @return a new stream channel
			 */
			static std::shared_ptr<Channel> create(const std::shared_ptr<AnswerStream> &stream);

			/**
			 * Push a QueryResult into this channel.
			 * @msg a QueryResult pointer.
			 */
			void push(const AnswerPtr &msg);

			/**
			 * Close the channel.
			 */
			void close();

			/**
			 * @return true if opened.
			 */
			bool isOpened() const;

			/**
			 * @return the id of this channel
			 */
			uint32_t id() const;

		protected:
			// the stream of this channel
			std::shared_ptr<AnswerStream> stream_;
			// iterator of this channel withing the stream
			std::list<std::shared_ptr<Channel>>::iterator iterator_;
			// flag indicating whether channel is open (i.e., no EOS received so far)
			std::atomic<bool> isOpened_;

			friend class AnswerStream;
		};

	protected:
		std::list<std::shared_ptr<Channel>> channels_;
		std::atomic<bool> isOpened_;
		std::mutex channel_mutex_;
		//uint32_t numCompletedChannels_;

		virtual void push(const Channel &channel, const AnswerPtr &msg);

		virtual void push(const AnswerPtr &msg) = 0;
	};
}

#endif //KNOWROB_QUERY_RESULT_STREAM_H_
