/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/MongoWatch.h"
#include "knowrob/db/mongo/MongoException.h"
#include "knowrob/db/mongo/bson_pl.h"

#include <sstream>
#include <iostream>

// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

// TODO: make configurable
#define WATCH_RATE_MS 200

MongoWatch::MongoWatch(mongoc_client_pool_t *client_pool)
: client_pool_(client_pool),
  isRunning_(false),
  thread_(NULL)
{
	watcher_opts_ = BCON_NEW(
		// maxAwaitTimeMS=0 for non blocking watcher
		"maxAwaitTimeMS", BCON_INT32(0),
		// TODO: what would be a good value
		"batchSize",      BCON_INT32(10)
	);
}

MongoWatch::~MongoWatch()
{
	// first let the thread terminate
	stopWatchThread();
	// then stop all watcher
	std::lock_guard<std::mutex> guard(lock_);
	for(std::map<std::string,MongoWatcher*>::iterator
			it=watcher_map_.begin(); it!=watcher_map_.end(); ++it)
	{
		MongoWatcher *watcher = it->second;
		delete watcher;
	}
	watcher_map_.clear();
	bson_destroy(watcher_opts_);
	watcher_opts_ = NULL;
}

void MongoWatch::startWatchThread()
{
	std::lock_guard<std::mutex> guard(lock_);
	if(!isRunning_ && !thread_) {
		isRunning_ = true;
		thread_ = new std::thread(&MongoWatch::loop, this);
	}
}

void MongoWatch::stopWatchThread()
{
	std::lock_guard<std::mutex> guard(lock_);
	if(isRunning_ && thread_) {
		isRunning_ = false;
		thread_->join();
		delete thread_;
		thread_ = NULL;
	}
}

void MongoWatch::watch(
		const char *db_name,
		const char *coll_name,
		const std::string &callback_goal,
		const PlTerm &query_term)
{
	// callback goal must be unique for now
	// TODO: better callback handling
	// TODO: print warning when watcher is overwritten
	unwatch(callback_goal);
	// add to map
	{
		MongoWatcher *watcher = new MongoWatcher(client_pool_, watcher_opts_,
				db_name, coll_name, callback_goal, query_term);
		std::lock_guard<std::mutex> guard(lock_);
		watcher_map_[callback_goal] = watcher;
	}
	// start the thread when the first watch is added
	startWatchThread();
}

void MongoWatch::unwatch(const std::string &callback_goal)
{
	std::map<std::string, MongoWatcher*>::iterator
		needle = watcher_map_.find(callback_goal);
	if(needle != watcher_map_.end()) {
		MongoWatcher *watcher = needle->second;
		std::lock_guard<std::mutex> guard(lock_);
		watcher_map_.erase(needle);
		delete watcher;
	}
	if(watcher_map_.empty()) {
		stopWatchThread();
	}
}

void MongoWatch::loop()
{
	auto next = std::chrono::system_clock::now();
	while(isRunning_) {
		call_watcher();

		next += std::chrono::milliseconds(WATCH_RATE_MS);
		std::this_thread::sleep_until(next);
	}
}

void MongoWatch::call_watcher()
{
	std::lock_guard<std::mutex> guard(lock_);
	for(std::map<std::string, MongoWatcher*>::iterator
			it=watcher_map_.begin(); it!=watcher_map_.end(); ++it)
	{
		it->second->next();
	}
}


MongoWatcher::MongoWatcher(
		mongoc_client_pool_t *pool,
		bson_t *opts,
		const char *db_name,
		const char *coll_name,
		const std::string &callback_goal,
		const PlTerm &query_term)
: callback_goal_(callback_goal),
  collection_(pool, db_name, coll_name),
  stream_(NULL)
{
	bson_t *query = bson_new();
	bson_error_t err;
	if(!bsonpl_concat(query, query_term, &err)) {
		bson_destroy(query);
		throw MongoException("invalid_query",err);
	}
	stream_ = mongoc_collection_watch(collection_(),query, NULL);
	bson_destroy(query);
}

MongoWatcher::~MongoWatcher()
{
	if(stream_!=NULL) {
		mongoc_change_stream_destroy(stream_);
		stream_ = NULL;
	}
}

void MongoWatcher::next()
{
	if(stream_==NULL) {
		return;
	}

	const bson_t *doc;
	if(mongoc_change_stream_next(stream_, &doc)) {
		PlTerm next_term = bson_to_term(doc);
		PlCall(callback_goal_.c_str(), PlTermv(next_term));
	}

	/*
	if(mongoc_change_stream_error_document(stream_, &error, &err_doc)) {
		if(!bson_empty (err_doc)) {
			fprintf(stderr, "Server Error: %s\n", bson_as_relaxed_extended_json (err_doc, NULL));
		} else {
			fprintf (stderr, "Client Error: %s\n", error.message);
		}
	}
	*/
}
