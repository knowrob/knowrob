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
  coll_(pool, db_name, coll_name)
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
	bson_t *doc = BCON_NEW("sort", "{", key, BCON_INT32(1), "}");
	bson_concat(opts_,doc);
	bson_destroy(doc);
}

void MongoCursor::descending(const char *key)
{
	bson_t *doc = BCON_NEW("sort", "{", key, BCON_INT32(-1), "}");
	bson_concat(opts_,doc);
	bson_destroy(doc);
}

void MongoCursor::filter(const PlTerm &query_term)
{
	bson_error_t err;
	bson_t *query=bson_new_from_term(query_term,&err);
	if(query) {
		bson_concat(query_,query);
		bson_destroy(query);
	}
}

bool MongoCursor::next(const bson_t **doc)
{
	if(cursor_==NULL) {
		cursor_ = mongoc_collection_find_with_opts(
		    coll_(), query_, opts_, NULL /* read_prefs */ );
	}
	// make sure cursor has no error
	bson_error_t err;
	if(mongoc_cursor_error(cursor_, &err)) {
		throw MongoException("cursor_error",err);
	}
	return mongoc_cursor_next(cursor_,doc);
}
