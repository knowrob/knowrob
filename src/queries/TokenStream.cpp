/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mutex>
#include <iostream>
#include <knowrob/Logger.h>
#include <knowrob/queries/TokenStream.h>
#include <knowrob/queries/QueryError.h>

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

void TokenStream::push(const Channel &channel, const TokenPtr &tok) {
	if (tok->indicatesEndOfEvaluation()) {
		bool doPushMsg = true;
		if (isOpened()) {
			// prevent channels from being created while processing EOS message
			std::lock_guard<std::mutex> lock(channel_mutex_);
			// close this stream if no channels are left
			// FIXME: what if a channel sends EOS twice?
			channels_.erase(channel.iterator_);
			doPushMsg = channels_.empty();
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
		  isOpened_(true) {
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
