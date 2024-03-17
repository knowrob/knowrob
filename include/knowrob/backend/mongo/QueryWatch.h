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
        explicit QueryWatch(mongoc_client_pool_t *client_pool);

        QueryWatch(const QueryWatch&) = delete;

        ~QueryWatch();

        long watch(const std::string_view &database,
                   const std::string_view &collection,
                   const bson_t *query,
                   const ChangeStreamCallback &callback);

        void unwatch(long watcher_id);

    protected:
        mongoc_client_pool_t *client_pool_;
        std::map<long, std::unique_ptr<ChangeStream>> watcher_map_;

        std::thread *thread_;
        bool isRunning_;
        std::mutex lock_;
        static std::atomic<long> id_counter_;

        void startWatchThread();
        void stopWatchThread();
        void loop();
    };
}

#endif //KNOWROB_MONGO_QUERY_WATCH_H
