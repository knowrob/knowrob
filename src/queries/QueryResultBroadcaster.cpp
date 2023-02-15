/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/QueryResultBroadcaster.h>

using namespace knowrob;

QueryResultBroadcaster::QueryResultBroadcaster()
: QueryResultStream()
{}

QueryResultBroadcaster::~QueryResultBroadcaster()
{
	if(isOpened()) {
		pushToBroadcast(QueryResultStream::eos());
	}
}

void QueryResultBroadcaster::addSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.push_back(subscriber);
}

void QueryResultBroadcaster::removeSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.remove(subscriber);
}

void QueryResultBroadcaster::push(const QueryResultPtr &item)
{
	pushToBroadcast(item);
}

void QueryResultBroadcaster::pushToBroadcast(const QueryResultPtr &item)
{
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}
