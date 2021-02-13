/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>
#include <iostream>

#include "knowrob/db/mongo/MongoInterface.h"
#include "knowrob/db/mongo/bson_pl.h"

PREDICATE(mng_collections,2) {
	MongoDatabase db_handle(MongoInterface::pool(), (char*)PL_A1);
	bson_error_t err;
	//
	char **strv;
	if ((strv = mongoc_database_get_collection_names_with_opts(
		    db_handle(), NULL /* opts */, &err))) {
		PlTail l(PL_A2);
		for (int i=0; strv[i]; i++) {
			l.append( strv[i] );
		}
		l.close();
		bson_strfreev(strv);
		return TRUE;
	}
	else {
		throw MongoException("collection_lookup_failed",err);
	}
}

PREDICATE(mng_distinct_values_json,4) {
	MongoDatabase db_handle(MongoInterface::pool(), (char*)PL_A1);
	char* coll_name = (char*)PL_A2;
	char* key = (char*)PL_A3;
	bson_error_t err;
	bson_t reply;
	//
	bson_t *command = BCON_NEW("distinct",
		BCON_UTF8(coll_name), "key", BCON_UTF8(key));
	bool success = mongoc_database_command_simple(
		db_handle(), command, NULL, &reply, &err);
	if(success) {
		char* str = bson_as_canonical_extended_json(&reply, NULL);
		PL_A4 = str;
		bson_free(str);
	}
	bson_destroy(command);
	return success;
}


PREDICATE(mng_drop_unsafe, 2) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	MongoInterface::get().drop(db_name,coll_name);
	return TRUE;
}

PREDICATE(mng_store, 3) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	MongoInterface::get().store(db_name,coll_name,PL_A3);
	return TRUE;
}

PREDICATE(mng_remove, 3) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	MongoInterface::get().remove(db_name,coll_name,PL_A3);
	return TRUE;
}

PREDICATE(mng_update, 4) {
	char* db_name     = (char*)PL_A1;
	char* coll_name   = (char*)PL_A2;
	MongoInterface::get().update(db_name,coll_name,PL_A3,PL_A4);
	return TRUE;
}

PREDICATE(mng_bulk_write, 3) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	MongoInterface::get().bulk_write(db_name,coll_name,PL_A3);
	return TRUE;
}

PREDICATE(mng_index_create_core, 3) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	MongoInterface::get().create_index(db_name,coll_name,PL_A3);
	return TRUE;
}

PREDICATE(mng_cursor_create, 3) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	PL_A3 = MongoInterface::cursor_create(db_name,coll_name)->id().c_str();
	return TRUE;
}

PREDICATE(mng_cursor_create, 4) {
	char* db_name   = (char*)PL_A1;
	char* coll_name = (char*)PL_A2;
	MongoCursor *cursor = MongoInterface::cursor_create(db_name,coll_name);
	cursor->filter(PL_A4);
	PL_A3 = cursor->id().c_str();
	return TRUE;
}

PREDICATE(mng_cursor_destroy, 1) {
	char* cursor_id = (char*)PL_A1;
	MongoInterface::cursor_destroy(cursor_id);
	return TRUE;
}

PREDICATE(mng_cursor_erase, 1) {
	char* cursor_id = (char*)PL_A1;
	return MongoInterface::cursor(cursor_id)->erase();
}

PREDICATE(mng_cursor_filter, 2) {
	char* cursor_id = (char*)PL_A1;
	MongoInterface::cursor(cursor_id)->filter(PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_aggregate, 2) {
	char* cursor_id = (char*)PL_A1;
	MongoInterface::cursor(cursor_id)->aggregate(PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_descending, 2) {
	char* cursor_id = (char*)PL_A1;
	char* key       = (char*)PL_A2;
	MongoInterface::cursor(cursor_id)->descending(key);
	return TRUE;
}

PREDICATE(mng_cursor_ascending, 2) {
	char* cursor_id = (char*)PL_A1;
	char* key       = (char*)PL_A2;
	MongoInterface::cursor(cursor_id)->ascending(key);
	return TRUE;
}

PREDICATE(mng_cursor_limit, 2) {
	char* cursor_id = (char*)PL_A1;
	int limit       = (int)PL_A2;
	MongoInterface::cursor(cursor_id)->limit(limit);
	return TRUE;
}

PREDICATE(mng_cursor_next_pairs, 2) {
	char* cursor_id = (char*)PL_A1;
	const bson_t *doc;
	if(MongoInterface::cursor(cursor_id)->next(&doc)) {
		PL_A2 = bson_to_term(doc);
		return TRUE;
	}
	else {
		return FALSE;
	}
}

PREDICATE(mng_cursor_next_json, 2) {
	char* cursor_id = (char*)PL_A1;
	const bson_t *doc;
	if(MongoInterface::cursor(cursor_id)->next(&doc)) {
		char* str = bson_as_canonical_extended_json(doc, NULL);
		PL_A2 = str;
		bson_free(str);
		return TRUE;
	}
	else {
		return FALSE;
	}
}

PREDICATE(mng_cursor_run, 1) {
	char* cursor_id = (char*)PL_A1;
	const bson_t *doc;
	if(MongoInterface::cursor(cursor_id)->next(&doc,true)) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}
