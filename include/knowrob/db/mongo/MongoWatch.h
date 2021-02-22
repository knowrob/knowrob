/* 
 * Copyright (c) 2021, Daniel Beßler
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
		const char *db_name,
		const char *coll_name,
		const std::string &callback_goal,
		const PlTerm &query_term);
	~MongoWatcher();

	bool next(long watcher_id);

protected:
	MongoCollection *collection_;
	std::string callback_goal_;
	mongoc_change_stream_t *stream_;
};

class MongoWatch {
public:
	MongoWatch(mongoc_client_pool_t *client_pool);
	~MongoWatch();

	long watch(
			const char *db_name,
			const char *coll_name,
			const std::string &callback_goal,
			const PlTerm &query_term);

	void unwatch(long watcher_id);

protected:
	mongoc_client_pool_t *client_pool_;
	std::map<long, MongoWatcher*> watcher_map_;

	std::thread *thread_;
	bool isRunning_;
	std::mutex lock_;
	long id_counter_;

	void startWatchThread();
	void stopWatchThread();
	void loop();
};

#endif //__KB_MONGO_WATCH_H__
