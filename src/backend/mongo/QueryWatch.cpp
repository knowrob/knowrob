/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/backend/mongo/QueryWatch.h"
#include "knowrob/backend/mongo/MongoException.h"
#include "knowrob/reasoner/mongolog/bson_pl.h"
#include <iostream>

using namespace knowrob::mongo;

std::atomic<long> QueryWatch::id_counter_ = 0;

QueryWatch::QueryWatch()
		: isRunning_(false),
		  thread_(nullptr),
		  watchRate_(200) {
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
		const std::shared_ptr<Collection> &collection,
		const bson_t *query,
		const ChangeStreamCallback &callback) {
	auto next_id = (id_counter_++);
	// add to map
	{
		std::lock_guard<std::mutex> guard(lock_);
		watcher_map_.emplace(next_id, std::make_unique<ChangeStream>(collection, query, callback));
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
		next += std::chrono::milliseconds(watchRate_);
		std::this_thread::sleep_until(next);
	}
}
