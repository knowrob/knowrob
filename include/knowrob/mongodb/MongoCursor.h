/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_CURSOR_H
#define KNOWROB_MONGO_CURSOR_H

#include <mongoc.h>
#include <string>
#include <memory>
#include <knowrob/mongodb/MongoCollection.h>

namespace knowrob {
    /**
     * A cursor that iterates over different results of a query.
     */
    class MongoCursor {
    public:
        explicit MongoCursor(const std::shared_ptr<MongoCollection> &collection);

        MongoCursor(const MongoCursor&) = delete;

        ~MongoCursor();

        const auto& id() { return id_; };

        void limit(unsigned int limit);

        void ascending(const char *key);

        void descending(const char *key);

        void filter(const bson_t *query_doc);

        void aggregate(const bson_t *query_doc);

        bool next(const bson_t **doc, bool ignore_empty=false);

        bool erase();

    private:
        std::shared_ptr<MongoCollection> collection_;
        mongoc_cursor_t *cursor_;
        bson_t *query_;
        bson_t *opts_;
        std::string id_;
        bool isAggregateQuery_;
    };
}

#endif //KNOWROB_MONGO_CURSOR_H
