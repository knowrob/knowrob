/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_COLLECTION_H
#define KNOWROB_MONGO_COLLECTION_H

#include <mongoc.h>
#include <string>
#include "BulkOperation.h"
#include "Document.h"
#include "Connection.h"

namespace knowrob::mongo {
	/**
	 * Configures an index key.
	 */
	struct IndexKey {
		explicit IndexKey(const char *key, const bool ascending = true)
				: value(key), ascending(true) {};
		const char *value;
		const bool ascending;
	};

	/**
	 * A named collection in Mongo DB.
	 */
	class Collection {
	public:
		Collection(
				const std::shared_ptr<Connection> &connection,
				const std::string_view &databaseName,
				const std::string_view &collectionName);

		Collection(const Collection &collection) = delete;

		~Collection();

		/**
		 * Append options to the session.
		 * @param opts some options
		 */
		void appendSession(bson_t *opts);

		/**
		 * @return the session handle.
		 */
		mongoc_client_session_t *session();

		/**
		 * @return the client pool of this collection.
		 */
		auto pool() { return connection_->pool_; }

		/**
		 * @return the connection of this collection.
		 */
		auto connection() { return connection_; }

		/**
		 * @return the mongo client managing of this collection.
		 */
		auto client() const { return client_; }

		/**
		 * @return the collection handle.
		 */
		auto coll() { return coll_; }

		/**
		 * @return the name of the collection.
		 */
		const auto &name() const { return name_; }

		/**
		 * @return the name of the database.
		 */
		const auto &dbName() const { return dbName_; }

		/**
		 * @return the URI string used to connect to the database.
		 */
		const auto &dbURI() const { return connection_->uri_string_; }

		/**
		 * Drop this collection, removing all document it contains.
		 */
		void drop();

		/**
		 * @param key the name of a field in mongo documents.
		 * @return a list of distinct values for the given key.
		 */
		std::vector<std::string> distinctValues(std::string_view key);

		/**
		 * Remove a single matching document from this collection.
		 * @param document a document pattern.
		 */
		void removeOne(const Document &document);

		/**
		 * Remove a single matching document from this collection.
		 * @param oid a document oid.
		 */
		void removeOne(const bson_oid_t &oid);

		/**
		 * Remove all matching JSON documents from this collection.
		 * @param document a document pattern.
		 */
		void removeAll(const Document &document);

		/**
		 * Store one document into this collection.
		 * @param document a JSON document.
		 */
		void storeOne(const Document &document);

		/**
		 * Update all existing documents matching the document pattern.
		 * @param query a document pattern.
		 * @param update the update document.
		 */
		void update(const Document &query, const Document &update, bool upsert = false);

		/**
		 * Evaluate an aggregation pipeline without forwarding any resulting
		 * documents. e.g. useful for pipelines with $merge stage.
		 * @param pipeline a pipeline
		 */
		void evalAggregation(const bson_t *pipeline);

		/**
		 * @return a new bulk operation.
		 */
		std::shared_ptr<BulkOperation> createBulkOperation();

		/**
		 * Create a search index where each key is sorted in ascending order.
		 * @param keys vector of keys
		 */
		void createAscendingIndex(const std::vector<const char *> &keys);

		/**
		 * Create a search index.
		 * @param keys vector of keys
		 */
		void createIndex(const std::vector<IndexKey> &keys);

		/**
		 * Create a search index for s/p/o fields.
		 */
		void createTripleIndex();

		/**
		 * @return true is this collection is empty.
		 */
		bool empty();

	private:
		std::shared_ptr<Connection> connection_;
		mongoc_client_t *client_;
		mongoc_client_session_t *session_;
		mongoc_collection_t *coll_;
		mongoc_database_t *db_;
		const std::string name_;
		const std::string dbName_;

		void remove(const Document &document, mongoc_remove_flags_t flag);

		void createIndex_internal(const bson_t &keys);
	};
}

#endif //KNOWROB_MONGO_COLLECTION_H
