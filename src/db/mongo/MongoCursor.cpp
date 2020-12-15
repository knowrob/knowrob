/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/MongoCursor.h"
#include "knowrob/db/mongo/MongoException.h"
#include "knowrob/db/mongo/bson_pl.h"

#include <sstream>
#include <iostream>

// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

MongoCursor::MongoCursor(
		mongoc_client_pool_t *pool,
		const char *db_name,
		const char *coll_name)
: cursor_(NULL),
  coll_(pool, db_name, coll_name),
  is_aggregate_query_(false)
{
	query_ = bson_new();
	opts_ = bson_new();
	// use pointer as id
	std::stringstream ss;
	ss << static_cast<const void*>(this);  
	id_ = ss.str(); 
}

MongoCursor::~MongoCursor()
{
	if(cursor_!=NULL) {
		mongoc_cursor_destroy(cursor_);
	}
	bson_destroy(query_);
	bson_destroy(opts_);
}

void MongoCursor::limit(unsigned int limit)
{
	BSON_APPEND_INT64(opts_, "limit", limit);
}

void MongoCursor::ascending(const char *key)
{
	static bson_t *doc = BCON_NEW("sort", "{", key, BCON_INT32(1), "}");
	bson_concat(opts_,doc);
}

void MongoCursor::descending(const char *key)
{
	static bson_t *doc = BCON_NEW("sort", "{", key, BCON_INT32(-1), "}");
	bson_concat(opts_,doc);
}

void MongoCursor::filter(const PlTerm &query_term)
{
	bson_error_t err;
	if(!bsonpl_concat(query_, query_term, &err)) {
		throw MongoException("invalid_term",err);
	}
}

void MongoCursor::aggregate(const PlTerm &query_term)
{
	bson_error_t err;
	is_aggregate_query_ = true;
	if(!bsonpl_concat(query_, query_term, &err)) {
		throw MongoException("invalid_term",err);
	}
//	else {
//		size_t len;
//		char *str = bson_as_canonical_extended_json (query_, &len);
//		std::cout << str << std::endl;
//		bson_free (str);
//	}
}

bool MongoCursor::next(const bson_t **doc)
{
	if(cursor_==NULL) {
		if(is_aggregate_query_) {
			cursor_ = mongoc_collection_aggregate(
				coll_(), MONGOC_QUERY_NONE, query_, opts_, NULL /* read_prefs */ );
		}
		else {
			cursor_ = mongoc_collection_find_with_opts(
			    coll_(), query_, opts_, NULL /* read_prefs */ );
		}
		// make sure cursor has no error after creation
		bson_error_t err1;
		if(mongoc_cursor_error(cursor_, &err1)) {
			throw MongoException("cursor_error",err1);
		}
	}
	// get next document
	if(!mongoc_cursor_next(cursor_,doc)) {
		// make sure cursor has no error after next has been called
		bson_error_t err2;
		if(mongoc_cursor_error(cursor_, &err2)) {
			throw MongoException("cursor_error",err2);
		}
		return false;
	}
	else {
		return true;
	}
}

bool MongoCursor::erase()
{
	bson_error_t err;
	bool success = mongoc_collection_delete_many(
		    coll_(), query_, opts_, NULL /* reply */, &err);
	if(!success) {
		throw MongoException("erase_error",err);
	}
	return true;
}
