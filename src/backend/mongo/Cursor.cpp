/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/mongo/Cursor.h"
#include "knowrob/backend/mongo/MongoException.h"
#include "knowrob/reasoner/mongolog/bson_pl.h"

#include <sstream>
#include <iostream>

using namespace knowrob::mongo;

Cursor::Cursor(const std::shared_ptr<Collection> &collection)
		: cursor_(nullptr),
		  collection_(collection),
		  isAggregateQuery_(false),
		  limit_(0) {
	query_ = bson_new();
	opts_ = bson_new();
	collection_->appendSession(opts_);
	// use pointer as id
	std::stringstream ss;
	ss << static_cast<const void *>(this);
	id_ = ss.str();
}

Cursor::~Cursor() {
	if (cursor_ != nullptr) {
		mongoc_cursor_destroy(cursor_);
	}
	bson_destroy(query_);
	bson_destroy(opts_);
}

void Cursor::limit(unsigned int limit) {
	limit_ = limit;
}

void Cursor::ascending(const char *key) {
	static bson_t *doc = BCON_NEW("sort", "{", key, BCON_INT32(1), "}");
	bson_concat(opts_, doc);
}

void Cursor::descending(const char *key) {
	static bson_t *doc = BCON_NEW("sort", "{", key, BCON_INT32(-1), "}");
	bson_concat(opts_, doc);
}

void Cursor::filter(const bson_t *query_doc) {
	bson_concat(query_, query_doc);
}

void Cursor::aggregate(const bson_t *query_doc) {
	isAggregateQuery_ = true;
	bson_concat(query_, query_doc);
}

bool Cursor::next(const bson_t **doc, bool ignore_empty) {
	if (cursor_ == nullptr) {
		if (isAggregateQuery_) {
			cursor_ = mongoc_collection_aggregate(
					collection_->coll(), MONGOC_QUERY_NONE, query_, opts_, nullptr /* read_prefs */ );
		} else {
			cursor_ = mongoc_collection_find_with_opts(
					collection_->coll(), query_, opts_, nullptr /* read_prefs */ );
		}
		if (limit_ > 0) {
			mongoc_cursor_set_limit(cursor_, limit_);
		}
		// make sure cursor has no error after creation
		bson_error_t err1;
		if (mongoc_cursor_error(cursor_, &err1)) {
			throw MongoException("cursor_error", err1);
		}
	}
	// get next document
	if (!mongoc_cursor_next(cursor_, doc)) {
		// make sure cursor has no error after next has been called
		bson_error_t err2;
		if (mongoc_cursor_error(cursor_, &err2)) {
			throw MongoException("cursor_error", err2);
		}
		return ignore_empty;
	} else {
		return true;
	}
}

bool Cursor::erase() {
	bson_error_t err;
	bool success = mongoc_collection_delete_many(
			collection_->coll(), query_, opts_, nullptr /* reply */, &err);
	if (!success) {
		throw MongoException("erase_error", err);
	}
	return true;
}
