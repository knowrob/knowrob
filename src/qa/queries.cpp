/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */
 
// SWI Prolog
#include <SWI-Prolog.h>
// KnowRob
#include <knowrob/qa/queries.h>

using namespace knowrob;

/******************************************/
/*************** Formulae *****************/
/******************************************/

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

/******************************************/
/************* query results **************/
/******************************************/

const QueryResult* QueryResult::NO_SOLUTION = new QueryResult();

QueryResult::QueryResult()
{}

bool QueryResult::hasSolution()
{
	return this != QueryResult::NO_SOLUTION;
}

void QueryResult::set(const Variable &var, const std::shared_ptr<Term> &term)
{
	mapping_[var] = term;
}

std::shared_ptr<Term> QueryResult::get(const Variable &var) const
{
	auto it = mapping_.find(var);
	if(it != mapping_.end()) {
		return it->second;
	}
	else {
		return std::shared_ptr<Term>();
	}
}


QueryResultStream::QueryResultStream(){}

QueryResultStream::~QueryResultStream()
{
	for(auto &x : subscriptions_) {
		x->removeSubscriber(this);
	}
}

bool QueryResultStream::isEOS(QueryResultPtr &item)
{
	return !item->hasSolution();
}


QueryResultBroadcast::QueryResultBroadcast()
: QueryResultStream(),
  numEOSPushed_(0)
{
}

QueryResultBroadcast::~QueryResultBroadcast()
{
	for(auto &x : subscribers_) {
		removeSubscriber(x);
	}
}

void QueryResultBroadcast::addSubscriber(QueryResultStream *subscriber)
{
	subscribers_.push_back(subscriber);
	subscriber->subscriptions_.push_back(this);
}

void QueryResultBroadcast::removeSubscriber(QueryResultStream *subscriber)
{
	subscribers_.remove(subscriber);
	subscriber->subscriptions_.remove(this);
}

void QueryResultBroadcast::push(QueryResultPtr &item)
{
	if(QueryResultStream::isEOS(item)) {
		// FIXME: broadcast EOS may never be published if a subscription channel is removed
		//        after EOS has been sent via this channel.
		numEOSPushed_ += 1;
		if(numEOSPushed_ < subscriptions_.size()) {
			return true;
		}
	}
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}


QueryResultQueue::QueryResultQueue()
: QueryResultStream()
{}

void QueryResultQueue::push(QueryResultPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		queue_.push(item);
	}
	cond_var_.notify_all();
}
		
QueryResultPtr& QueryResultQueue::front()
{
	std::unique_lock<std::mutex> lock(mutex_);
	// wait until QueryResultPtr at index is available
	cond_var_.wait(lock, [&]{ return !queue_.empty(); });
	return queue_.front();
}

void QueryResultQueue::pop()
{
	std::lock_guard<std::mutex> lock(mutex_);
	queue_.pop();
}

