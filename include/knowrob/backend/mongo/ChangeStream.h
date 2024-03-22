/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

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
    using ChangeStreamCallback = std::function<void(const bson_wrapper_ptr&)>;

    /**
     * A stream of query results that invokes a callback whenever a new
     * result is found.
     */
    class ChangeStream {
    public:
        ChangeStream(
                const std::shared_ptr<Collection> &collection,
                const bson_t *query,
                ChangeStreamCallback callback);

        ChangeStream(const ChangeStream&) = delete;

        ~ChangeStream();

        bool next();

    protected:
        std::shared_ptr<Collection> collection_;
        ChangeStreamCallback callback_;
        mongoc_change_stream_t *stream_;
        bson_wrapper_ptr next_ptr_;
    };

} // knowrob::mongo

#endif //KNOWROB_MONGO_CHANGE_STREAM_H
