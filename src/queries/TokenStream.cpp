/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mutex>
#include <knowrob/Logger.h>
#include <knowrob/queries/TokenStream.h>
#include <knowrob/queries/QueryError.h>
#include "knowrob/integration/python/utils.h"
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/queries/TokenBroadcaster.h"
#include "knowrob/queries/TokenBuffer.h"

using namespace knowrob;

TokenStream::TokenStream()
		: isOpened_(true) {}

TokenStream::~TokenStream() {
	close();
}

void TokenStream::close() {
	std::list<std::shared_ptr<Channel>> channels;
	{
		std::lock_guard<std::mutex> lock(channel_mutex_);
		// make a copy of the channels list to prevent that the channels list is modified
		// while closing the channels
		channels = channels_;
	}
	// allow the channels to generate EOS messages for clean shutdown
	for (auto &channel: channels) {
		channel->close();
	}
	{
		// Finally mark the stream as closed
		std::lock_guard<std::mutex> lock(channel_mutex_);
		channels_.clear();
		isOpened_ = false;
	}
}

bool TokenStream::isOpened() const {
	return isOpened_;
}

void TokenStream::push(Channel &channel, const TokenPtr &tok) {
	if (tok->indicatesEndOfEvaluation()) {
		bool doPushMsg = false;
		{
			// prevent channels from being created or closed while processing EOS message
			std::lock_guard<std::mutex> lock(channel_mutex_);
			if (isOpened()) {
				if (channel.hasValidIterator()) {
					// close this stream if no channels are left
					channels_.erase(channel.iterator_);
					channel.invalidateIterator();
					doPushMsg = channels_.empty();
					isOpened_ = !doPushMsg;
				} else {
					KB_WARN("ignoring attempt to write to a channel with a singular iterator.");
				}
			}
		}
		// send EOS on this stream if no channels are left
		if (doPushMsg) {
			push(tok);
		}
	} else if (!isOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	} else {
		push(tok);
	}
}


TokenStream::Channel::Channel(const std::shared_ptr<TokenStream> &stream)
		: stream_(stream),
		  isOpened_(true),
		  hasValidIterator_(true) {
}

TokenStream::Channel::~Channel() {
	close();
}

std::shared_ptr<TokenStream::Channel> TokenStream::Channel::create(
		const std::shared_ptr<TokenStream> &stream) {
	// prevent the stream from being closed and to modify the channels list,
	// as we are about to add a new channel to the list.
	std::lock_guard<std::mutex> lock1(stream->channel_mutex_);
	if (stream->isOpened()) {
		auto channel = std::make_shared<TokenStream::Channel>(stream);
		stream->channels_.push_back(channel);
		channel->iterator_ = stream->channels_.end();
		--channel->iterator_;
		return channel;
	} else {
		throw QueryError("cannot create a channel of a closed stream");
	}
}

void TokenStream::Channel::close() {
	// prevent channels from being closed while other channel operations are in progress.
	// also avoid close being called multiple times at the same time.
	std::unique_lock<std::shared_mutex> lock(mutex_);
	if (isOpened()) {
		isOpened_ = false;
		if (stream_->isOpened()) {
			auto s = stream_;
			stream_ = {};
			lock.unlock();
			s->push(*this, EndOfEvaluation::get());
		}
	}
}

uint32_t TokenStream::Channel::id() const {
	return reinterpret_cast<std::uintptr_t>(this);
}

void TokenStream::Channel::push(const TokenPtr &tok) {
	// prevent channels from being closed while push operations are in progress
	// note: this is a shared lock, i.e., multiple push operations can be performed in parallel.
	std::unique_lock<std::shared_mutex> lock(mutex_);
	if (isOpened()) {
		auto s = stream_;
		if (tok->indicatesEndOfEvaluation()) {
			isOpened_ = false;
			stream_ = {};
		}
		lock.unlock();
		s->push(*this, tok);
	} else if (!tok->indicatesEndOfEvaluation()) {
		KB_WARN("message pushed to closed stream {}", reinterpret_cast<std::uintptr_t>(this));
	}
}

bool TokenStream::Channel::isOpened() const {
	return isOpened_;
}

namespace knowrob::py {
	template<>
	void createType<TokenStream>() {
		using namespace boost::python;
		createType<Token>();
		class_<TokenStream, std::shared_ptr<TokenStream>, boost::noncopyable>
				("TokenStream", no_init)
				.def("isOpened", &TokenStream::isOpened);
		class_<TokenStream::Channel, std::shared_ptr<TokenStream::Channel>, boost::noncopyable>
				("TokenChannel", no_init)
				.def("create", &TokenStream::Channel::create).staticmethod("create")
				.def("push", with<no_gil>(&TokenStream::Channel::push))
				.def("close", with<no_gil>(&TokenStream::Channel::close))
				.def("isOpened", &TokenStream::Channel::isOpened)
				.def("id", &TokenStream::Channel::id);
		createType<TokenQueue>();
		createType<TokenBroadcaster>();
		createType<TokenBuffer>();
	}
}
