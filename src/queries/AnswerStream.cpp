/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mutex>
#include <iostream>
#include <knowrob/Logger.h>
#include <knowrob/queries/AnswerStream.h>
#include <knowrob/queries/QueryError.h>

using namespace knowrob;

AnswerStream::AnswerStream()
: isOpened_(true)
{}

AnswerStream::~AnswerStream()
{
	close();
}

void AnswerStream::close()
{
	for(auto &channel : channels_) {
		channel->close();
	}
	{
		std::lock_guard<std::mutex> lock(channel_mutex_);
		channels_.clear();
		isOpened_ = false;
	}
}

bool AnswerStream::isEOS(const AnswerPtr &item)
{
	return (item.get() == eos().get());
}

AnswerPtr& AnswerStream::bos()
{
	static auto msg = std::make_shared<const Answer>();
	return msg;
}

AnswerPtr& AnswerStream::eos()
{
	static auto msg = std::make_shared<const Answer>();
	return msg;
}

bool AnswerStream::isOpened() const
{
	return isOpened_;
}

void AnswerStream::push(const Channel &channel, const AnswerPtr &item)
{
	if(AnswerStream::isEOS(item)) {
		bool doPushMsg = true;
		if(isOpened()) {
			// prevent channels from being created while processing EOS message
			std::lock_guard<std::mutex> lock(channel_mutex_);
			// close this stream if no channels are left
			// FIXME: what if a channel sends EOS twice?
			channels_.erase(channel.iterator_);
			doPushMsg = channels_.empty();
		}
		// send EOS on this stream if no channels are left
		if(doPushMsg) {
			push(item);
			isOpened_ = false;
		}
	}
	else if(!isOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	}
	else {
		push(item);
	}
}


AnswerStream::Channel::Channel(const std::shared_ptr<AnswerStream> &stream)
: stream_(stream),
  isOpened_(true)
{
}

AnswerStream::Channel::~Channel()
{
	close();
}

std::shared_ptr<AnswerStream::Channel> AnswerStream::Channel::create(
		const std::shared_ptr<AnswerStream> &stream)
{
	std::lock_guard<std::mutex> lock(stream->channel_mutex_);
	if(stream->isOpened()) {
		auto channel = std::make_shared<AnswerStream::Channel>(stream);
		stream->channels_.push_back(channel);
		channel->iterator_ = stream->channels_.end();
		--channel->iterator_;
		return channel;
	}
	else {
		throw QueryError("cannot create a channel of a closed stream");
	}
}

void AnswerStream::Channel::close()
{
	if(isOpened()) {
		stream_->push(*this, AnswerStream::eos());
		isOpened_ = false;
		stream_ = {};
	}
}

uint32_t AnswerStream::Channel::id() const
{
	return reinterpret_cast<std::uintptr_t>(this);
}

void AnswerStream::Channel::push(const AnswerPtr &msg)
{
	if(isOpened()) {
		stream_->push(*this, msg);
		if(AnswerStream::isEOS(msg)) {
			isOpened_ = false;
			stream_ = {};
		}
	}
	else if(!AnswerStream::isEOS(msg)) {
		KB_WARN("message pushed to closed stream {}", reinterpret_cast<std::uintptr_t>(this));
	}
}

bool AnswerStream::Channel::isOpened() const
{
	return isOpened_;
}
