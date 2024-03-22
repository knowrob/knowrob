/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/mongo/ChangeStream.h"
#include "knowrob/backend/mongo/MongoException.h"

using namespace knowrob::mongo;

ChangeStream::ChangeStream(
		const std::shared_ptr<Collection> &collection,
		const bson_t *query,
		ChangeStreamCallback callback)
		: collection_(collection),
		  stream_(nullptr),
		  callback_(std::move(callback)),
		  next_ptr_() {
	// connect and append session ID to options
	bson_t *opts = BCON_NEW(
			"maxAwaitTimeMS", BCON_INT32(1),            // the watcher should be non-blocking
			"fullDocument", BCON_UTF8("updateLookup")   // always fetch full document
	);
	collection_->appendSession(opts);
	bson_destroy(opts);
	// create the stream object
	stream_ = mongoc_collection_watch(collection_->coll(), query, opts);
}

ChangeStream::~ChangeStream() {
	if (stream_ != nullptr) {
		mongoc_change_stream_destroy(stream_);
		stream_ = nullptr;
	}
}

bool ChangeStream::next() {
	if (stream_ == nullptr) {
		// stream had an error before
		return false;
	}

	// try retrieving next document
	const bson_t *doc;
	if (mongoc_change_stream_next(stream_, &doc)) {
		next_ptr_.bson = doc;
		callback_(next_ptr_);
		return true;
	}

	// check if stream has an error
	const bson_t *err_doc;
	bson_error_t error;
	if (mongoc_change_stream_error_document(stream_, &error, &err_doc)) {
		mongoc_change_stream_destroy(stream_);
		stream_ = nullptr;
		throw MongoException("watch_error", error);
	}

	return false;
}
