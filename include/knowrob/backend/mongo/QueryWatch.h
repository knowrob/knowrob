/* 
 * Copyright (c) 2021, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_QUERY_WATCH_H
#define KNOWROB_MONGO_QUERY_WATCH_H

#include <mongoc.h>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <map>
#include <knowrob/backend/mongo/ChangeStream.h>

namespace knowrob::mongo {
    /**
     * Keeps track over time of query results and notifies a callback
     * for each new result.
     */
    class QueryWatch {
    public:
		QueryWatch();

        QueryWatch(const QueryWatch&) = delete;

        ~QueryWatch();

        /**
         * The query watch actively polls change streams in this interval.
         * @param rate the rate in milliseconds.
         */
        void setWatchRate(uint32_t rate) { watchRate_ = rate; }

        /**
		 * Watch for changes in a collection.
		 * @param collection the collection to watch.
		 * @param query the query to watch.
		 * @param callback the callback to invoke for each change.
		 * @return a watcher id that can be used to unwatch.
		 */
        long watch(const std::shared_ptr<Collection> &collection,
                   const bson_t *query,
                   const ChangeStreamCallback &callback);

		/**
		 * Stop watching a collection.
		 * @param watcher_id the watcher id returned by watch.
		 */
        void unwatch(long watcher_id);

    protected:
        std::map<long, std::unique_ptr<ChangeStream>> watcher_map_;

        std::thread *thread_;
        bool isRunning_;
        std::mutex lock_;
		uint32_t watchRate_;
        static std::atomic<long> id_counter_;

        void startWatchThread();
        void stopWatchThread();
        void loop();
    };
}

#endif //KNOWROB_MONGO_QUERY_WATCH_H
