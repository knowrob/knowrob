/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/backend/mongo/QueryWatch.h"
#include "knowrob/backend/mongo/MongoException.h"
#include "knowrob/reasoner/mongolog/bson_pl.h"
#include <iostream>
#include <utility>
// SWI Prolog
#define PL_SAFE_ARG_MACROS

#include <SWI-cpp.h>

// TODO: make configurable
#define WATCH_RATE_MS 200

using namespace knowrob::mongo;

std::atomic<long> QueryWatch::id_counter_ = 0;

QueryWatch::QueryWatch(mongoc_client_pool_t *client_pool)
		: client_pool_(client_pool),
		  isRunning_(false),
		  thread_(nullptr) {
	// TODO: consider using one thread per collection,
	//       _and_ one stream per collection.
	//       then build a composite $match covering all
	//       watch requests.
	//       finally do some processing client side to decide
	//       which callback must be notified.
	// TODO: special handling for remove events as they do not
	//       provide the values $match cannot be used
}

QueryWatch::~QueryWatch() {
	// first let the thread terminate
	stopWatchThread();
	// then stop all watcher
	watcher_map_.clear();
}

void QueryWatch::startWatchThread() {
	std::lock_guard<std::mutex> guard(lock_);
	if (!isRunning_ && !thread_) {
		isRunning_ = true;
		thread_ = new std::thread(&QueryWatch::loop, this);
	}
}

void QueryWatch::stopWatchThread() {
	std::lock_guard<std::mutex> guard(lock_);
	if (isRunning_ && thread_) {
		isRunning_ = false;
		thread_->join();
		delete thread_;
		thread_ = nullptr;
	}
}

long QueryWatch::watch(
		const std::string_view &database,
		const std::string_view &collection,
		const bson_t *query,
		const ChangeStreamCallback &callback) {
	auto next_id = (id_counter_++);
	// add to map
	{
		std::lock_guard<std::mutex> guard(lock_);
		watcher_map_.emplace(next_id, std::make_unique<ChangeStream>(
				client_pool_, database, collection,
				next_id, query, callback));
	}
	// start the thread when the first watch is added
	startWatchThread();
	return next_id;
}

void QueryWatch::unwatch(long watcher_id) {
	auto needle = watcher_map_.find(watcher_id);
	if (needle != watcher_map_.end()) {
		std::lock_guard<std::mutex> guard(lock_);
		watcher_map_.erase(needle);
	}
	if (watcher_map_.empty()) {
		stopWatchThread();
	}
}

void QueryWatch::loop() {
	// bind a Prolog engine to this thread.
	// this is needed as callback's are predicates in the
	// Prolog knowledge base.
	if (!PL_thread_attach_engine(nullptr)) {
		KB_ERROR("failed to attach engine!");
		isRunning_ = false;
	}
	// loop as long isRunning_=true
	auto next = std::chrono::system_clock::now();
	while (isRunning_) {
		{
			std::lock_guard<std::mutex> guard(lock_);
			for (auto &it: watcher_map_) {
				try {
					it.second->next();
				}
				catch (MongoException &exc) {
					KB_WARN("exception in mongo watch: {}", exc.what());
				}
			}
		}
		// try to run at constant rate
		next += std::chrono::milliseconds(WATCH_RATE_MS);
		std::this_thread::sleep_until(next);
	}
}
