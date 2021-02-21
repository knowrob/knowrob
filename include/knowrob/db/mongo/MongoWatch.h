/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KB_MONGO_WATCH_H__
#define __KB_MONGO_WATCH_H__

#include <mongoc.h>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <map>

// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>
// knowrob_mongo
#include <knowrob/db/mongo/MongoCollection.h>

class MongoWatcher {
public:
	MongoWatcher(
		mongoc_client_pool_t *pool,
		bson_t *opts,
		const char *db_name,
		const char *coll_name,
		const std::string &callback_goal,
		const PlTerm &query_term);
	~MongoWatcher();

	void next();

protected:
	MongoCollection collection_;
	std::string callback_goal_;
	mongoc_change_stream_t *stream_;
};

class MongoWatch {
public:
	MongoWatch(mongoc_client_pool_t *client_pool);
	~MongoWatch();

	void watch(const char *db_name, const char *coll_name, const std::string &callback_goal, const PlTerm &query_term);

	void unwatch(const std::string &callback_goal);

protected:
	mongoc_client_pool_t *client_pool_;
	std::map<std::string, MongoWatcher*> watcher_map_;
	bson_t *watcher_opts_;

	std::thread *thread_;
	bool isRunning_;
	std::mutex lock_;

	void startWatchThread();
	void stopWatchThread();
	void loop();
	void call_watcher();
};

#endif //__KB_MONGO_WATCH_H__
