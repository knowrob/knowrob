//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_MONGO_CHANGE_STREAM_H
#define KNOWROB_MONGO_CHANGE_STREAM_H

#include <mongoc.h>
#include <string>
#include <memory>
#include "Collection.h"
#include "bson.h"
#include "bson-helper.h"

namespace knowrob::mongo {
    /**
     * Called for each result of a change stream.
     */
    using ChangeStreamCallback = std::function<void(long, const bson_wrapper_ptr&)>;

    /**
     * A stream of query results that invokes a callback whenever a new
     * result is found.
     */
    class ChangeStream {
    public:
        ChangeStream(
                mongoc_client_pool_t *pool,
                const std::string_view &databaseName,
                const std::string_view &collectionName,
                long queryID,
                const bson_t *query,
                ChangeStreamCallback callback);

        ChangeStream(const ChangeStream&) = delete;

        ~ChangeStream();

        bool next();

    protected:
        std::unique_ptr<Collection> collection_;
        ChangeStreamCallback callback_;
        mongoc_change_stream_t *stream_;
        bson_wrapper_ptr next_ptr_;
        long queryID_;
    };

} // knowrob::mongo

#endif //KNOWROB_MONGO_CHANGE_STREAM_H
