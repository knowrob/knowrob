/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_COLLECTION_H
#define KNOWROB_MONGO_COLLECTION_H

#include <mongoc.h>
#include <string>
#include "MongoBulkOperation.h"
#include "MongoDocument.h"

namespace knowrob {
    /**
     * Configures an index key.
     */
    struct MongoIndexKey {
        explicit MongoIndexKey(const char *key, const bool ascending=true)
                : value(key), ascending(true) {};
        const char *value;
        const bool ascending;
    };

    /**
     * A named collection in Mongo DB.
     */
    class MongoCollection {
    public:
        MongoCollection(
                mongoc_client_pool_t *pool,
                const std::string &db_name,
                const std::string &coll_name);

        MongoCollection(
                mongoc_client_pool_t *pool,
                mongoc_client_t *client,
                mongoc_client_session_t *session,
                const std::string &db_name,
                const std::string &coll_name);

        ~MongoCollection();

        /**
         * Append options to the session.
         * @param opts some options
         */
        void appendSession(bson_t *opts);

        /**
         * @return the session handle.
         */
        auto session() { return session_; }

        auto pool() { return pool_; }

        /**
         * @return the collection handle.
         */
        auto coll() { return coll_; }

        /**
         * @return the collection handle.
         */
        auto operator()() { return coll_; }

        /**
         * @return the name of the collection.
         */
        const auto& name() const { return name_; }

        const auto& dbName() const { return dbName_; }

        auto client() const { return client_; }

        /**
         * Drop this collection, removing all document it contains.
         */
        void drop();

        /**
         * Remove a single matching document from this collection.
         * @param document a document pattern.
         */
        void removeOne(const MongoDocument &document);

        /**
         * Remove all matching JSON documents from this collection.
         * @param document a document pattern.
         */
        void removeAll(const MongoDocument &document);

        /**
         * Store one document into this collection.
         * @param document a JSON document.
         */
        void storeOne(const MongoDocument &document);

        /**
         * Update all existing documents matching the document pattern.
         * @param query a document pattern.
         * @param update the update document.
         */
        void update(const MongoDocument &query, const MongoDocument &update);

        void evalAggregation(const MongoDocument &pipeline);

        /**
         * @return a new bulk operation.
         */
        std::shared_ptr<MongoBulkOperation> createBulkOperation();

        /**
         * Create a search index where each key is sorted in ascending order.
         * @param keys vector of keys
         */
        void createAscendingIndex(const std::vector<const char*> &keys);

        /**
         * Create a search index.
         * @param keys vector of keys
         */
        void createIndex(const std::vector<MongoIndexKey> &keys);

    private:
        mongoc_client_pool_t *pool_;
        mongoc_client_t *client_;
        mongoc_client_session_t *session_;
        mongoc_collection_t *coll_;
        mongoc_database_t *db_;
        const std::string name_;
        const std::string dbName_;
        bool isSessionOwned_;

        void remove(const MongoDocument &document, mongoc_remove_flags_t flag);
        void createIndex_internal(const bson_t &keys);
    };
}

#endif //KNOWROB_MONGO_COLLECTION_H
