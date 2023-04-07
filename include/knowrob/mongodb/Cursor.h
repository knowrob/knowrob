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
#include <knowrob/mongodb/Collection.h>

namespace knowrob::mongo {
    /**
     * A cursor that iterates over different results of a query.
     */
    class Cursor {
    public:
        explicit Cursor(const std::shared_ptr<Collection> &collection);

        Cursor(const std::shared_ptr<Collection> &collection, bson_t *query, bool aggregate);

        Cursor(const Cursor&) = delete;

        ~Cursor();

        /**
         * @return the unique id of this cursor.
         */
        const auto& id() { return id_; };

        /**
         * Limit results produced by this cursor.
         * @param limit the maximum number of results.
         */
        void limit(unsigned int limit);

        /**
         * Sort results in ascending order.
         * @param key a field name in result documents.
         */
        void ascending(const char *key);

        /**
         * Sort results in descending order.
         * @param key a field name in result documents.
         */
        void descending(const char *key);

        /**
         * Filter results of the cursor by the pattern provided.
         * @param query_doc a query document.
         */
        void filter(const bson_t *query_doc);

        /**
         * Run an aggregation pipeline to obtain results for this cursor.
         * @param query_doc
         */
        void aggregate(const bson_t *query_doc);

        /**
         * Start the query if it has not been started yet and generate the next result document.
         * @param doc pointer to result document.
         * @param ignore_empty
         * @return true on success
         */
        bool next(const bson_t **doc, bool ignore_empty=false);

        /**
         * Erase all documents that are result documents of this cursor.
         * @return true on success.
         */
        bool erase();

    private:
        std::shared_ptr<Collection> collection_;
        mongoc_cursor_t *cursor_;
        bson_t *query_;
        bson_t *opts_;
        std::string id_;
        bool isAggregateQuery_;
    };
}

#endif //KNOWROB_MONGO_CURSOR_H
