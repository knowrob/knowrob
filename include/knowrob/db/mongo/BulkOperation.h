/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_BULK_OPERATION_H
#define KNOWROB_MONGO_BULK_OPERATION_H

#include <mongoc.h>
#include <memory>

namespace knowrob::mongo {
	/**
	 * Provides an abstraction for submitting multiple write operations as a single batch.
	 */
	class BulkOperation {
	public:
		/**
		 * Default constructor.
		 * Note that the pointer is owned by this object afterwards,
		 * and it will take care of freeing its memory.
		 * @param handle a handle to the mongoc_bulk_operation_t pointer.
		 */
		explicit BulkOperation(mongoc_bulk_operation_t *handle);

		BulkOperation(const BulkOperation &) = delete;

		~BulkOperation();

		/**
		 * Add an insertion operation to this batch.
		 * @param document a document.
		 */
		void pushInsert(const bson_t *document);

		/**
		 * Add a removal operation to this batch.
		 * @param document a document pattern.
		 */
		void pushRemoveAll(const bson_t *document);

		/**
		 * Add a removal operation to this batch.
		 * @param document a document pattern.
		 */
		void pushRemoveOne(const bson_t *document);

		/**
		 * Add an update operation to this batch.
		 * @param query a document pattern.
		 * @param update a update document.
		 */
		void pushUpdate(bson_t *query, bson_t *update);

		/**
		 * Execute this bulk operation.
		 * Note that a bulk operation can only be executed once.
		 */
		void execute();

	protected:
		mongoc_bulk_operation_t *handle_;

		void validateBulkHandle();
	};

} // knowrob

#endif //KNOWROB_MONGO_BULK_OPERATION_H
