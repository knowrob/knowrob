/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mutex>
#include <knowrob/Logger.h>
#include <knowrob/queries/QueryResultStream.h>
#include <knowrob/queries/QueryError.h>

using namespace knowrob;

QueryResultStream::QueryResultStream()
: isOpened_(true)
{}

QueryResultStream::~QueryResultStream()
{
	close();
}

void QueryResultStream::close()
{
	if(isOpened()) {
		std::lock_guard<std::mutex> lock(channel_mutex_);
		for(auto &c : channels_) {
			c->isOpened_ = false;
		}
		channels_.clear();
		isOpened_ = false;
	}
}

bool QueryResultStream::isEOS(const QueryResultPtr &item)
{
	return (item.get() == eos().get());
}

QueryResultPtr& QueryResultStream::bos()
{
	static auto msg = std::make_shared<const QueryResult>();
	return msg;
}

QueryResultPtr& QueryResultStream::eos()
{
	static auto msg = std::make_shared<const QueryResult>();
	return msg;
}

bool QueryResultStream::isOpened() const
{
	return isOpened_;
}

void QueryResultStream::push(const Channel &channel, const QueryResultPtr &item)
{
	if(QueryResultStream::isEOS(item)) {
		bool doPushMsg; {
			// prevent channels from being created while processing EOS message
			std::lock_guard<std::mutex> lock(channel_mutex_);
			// remove channel once EOS is reached
			channels_.erase(channel.iterator_);
			// auto-close this stream if no channels are left
			if(channels_.empty() && isOpened()) {
				isOpened_ = false;
				doPushMsg = true;
			}
			else {
				doPushMsg = false;
			}
		}
		// send EOS on this stream if no channels are left
		if(doPushMsg) {
			push(item);
		}
	}
	else if(!isOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	}
	else {
		push(item);
	}
}


QueryResultStream::Channel::Channel(const std::shared_ptr<QueryResultStream> &stream)
: stream_(stream),
  isOpened_(true)
{
}

QueryResultStream::Channel::~Channel()
{
	close();
}

std::shared_ptr<QueryResultStream::Channel> QueryResultStream::Channel::create(
		const std::shared_ptr<QueryResultStream> &stream)
{
	std::lock_guard<std::mutex> lock(stream->channel_mutex_);
	if(stream->isOpened()) {
		auto channel = std::make_shared<QueryResultStream::Channel>(stream);
		stream->channels_.push_back(channel);
		channel->iterator_ = stream->channels_.end();
		--channel->iterator_;
		return channel;
	}
	else {
		throw QueryError("cannot create a channel of a closed stream");
	}
}

void QueryResultStream::Channel::close()
{
	if(isOpened()) {
		stream_->push(*this, QueryResultStream::eos());
		isOpened_ = false;
	}
}

uint32_t QueryResultStream::Channel::id() const
{
	return reinterpret_cast<std::uintptr_t>(this);
}

void QueryResultStream::Channel::push(const QueryResultPtr &msg)
{
	if(isOpened()) {
		stream_->push(*this, msg);
		if(QueryResultStream::isEOS(msg)) {
			isOpened_ = false;
		}
	}
	else if(!QueryResultStream::isEOS(msg)) {
		KB_WARN("message pushed to closed stream {}", reinterpret_cast<std::uintptr_t>(this));
	}
}

bool QueryResultStream::Channel::isOpened() const
{
	return isOpened_;
}
