/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mutex>
#include <iostream>
#include <knowrob/Logger.h>
#include <knowrob/queries/TokenStream.h>
#include <knowrob/queries/QueryError.h>
#include "knowrob/py/utils.h"

using namespace knowrob;

TokenStream::TokenStream()
		: isOpened_(true) {}

TokenStream::~TokenStream() {
	close();
}

void TokenStream::close() {
	for (auto &channel: channels_) {
		channel->close();
	}
	{
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
		bool doPushMsg = true;
		if (isOpened()) {
			if (channel.hasValidIterator()) {
				// prevent channels from being created while processing EOS message
				std::lock_guard<std::mutex> lock(channel_mutex_);
				// close this stream if no channels are left
				channels_.erase(channel.iterator_);
				channel.invalidateIterator();
				doPushMsg = channels_.empty();
			} else {
				KB_WARN("ignoring attempt to write to a channel with a singular iterator.");
			}
		}
		// send EOS on this stream if no channels are left
		if (doPushMsg) {
			push(tok);
			isOpened_ = false;
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
	std::lock_guard<std::mutex> lock(stream->channel_mutex_);
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
	if (isOpened()) {
		stream_->push(*this, EndOfEvaluation::get());
		isOpened_ = false;
		stream_ = {};
	}
}

uint32_t TokenStream::Channel::id() const {
	return reinterpret_cast<std::uintptr_t>(this);
}

void TokenStream::Channel::push(const TokenPtr &tok) {
	if (isOpened()) {
		stream_->push(*this, tok);
		if (tok->indicatesEndOfEvaluation()) {
			isOpened_ = false;
			stream_ = {};
		}
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
		class_<TokenStream, std::shared_ptr<TokenStream>, boost::noncopyable>
				("TokenStream", no_init)
				.def("isOpened", &TokenStream::isOpened);
		class_<TokenStream::Channel, std::shared_ptr<TokenStream::Channel>, boost::noncopyable>
				("TokenChannel", no_init)
				.def("create", &TokenStream::Channel::create)
				.def("push", &TokenStream::Channel::push)
				.def("close", &TokenStream::Channel::close)
				.def("isOpened", &TokenStream::Channel::isOpened)
				.def("id", &TokenStream::Channel::id);
	}
}
