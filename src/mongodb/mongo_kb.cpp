/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>
// STD
#include <iostream>
// KnowRob
#include <knowrob/mongodb/MongoInterface.h>
#include <knowrob/mongodb/bson_pl.h>

PREDICATE(mng_collections,2) {
	auto db_handle = MongoInterface::get().connect(PL_A1);
	bson_error_t err;
	char **strv;
	if ((strv = mongoc_database_get_collection_names_with_opts(
			db_handle->db(), nullptr /* opts */, &err))) {
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
	auto db_handle = MongoInterface::get().connect(PL_A1);
	char* coll_name = (char*)PL_A2;
	char* key = (char*)PL_A3;
	bson_error_t err;
	bson_t reply;
	//
	bson_t *command = BCON_NEW("distinct", BCON_UTF8(coll_name), "key", BCON_UTF8(key));
	bool success = mongoc_database_command_simple(
			db_handle->db(), command, nullptr, &reply, &err);
	if(success) {
		char* str = bson_as_canonical_extended_json(&reply, nullptr);
		PL_A4 = str;
		bson_free(str);
	}
	bson_destroy(command);
	return success;
}

PREDICATE(mng_index_create_core, 3) {
	return MongoInterface::get().connect(PL_A1)->create_index((char*)PL_A2,PL_A3);
}


PREDICATE(mng_drop_unsafe, 2) {
	return MongoInterface::get().connect(PL_A1, (char*)PL_A2)->drop();
}

PREDICATE(mng_store, 3) {
	return MongoInterface::get().connect(PL_A1, (char*)PL_A2)->store(PL_A3);
}

PREDICATE(mng_remove, 3) {
	return MongoInterface::get().connect(PL_A1, (char*)PL_A2)->remove(PL_A3);
}

PREDICATE(mng_update, 4) {
	return MongoInterface::get().connect(PL_A1, (char*)PL_A2)->update(PL_A3, PL_A4);
}

PREDICATE(mng_bulk_write, 3) {
	return MongoInterface::get().connect(PL_A1, (char*)PL_A2)->bulk_write(PL_A3);
}


PREDICATE(mng_watch, 5) {
	char* coll_name = (char*)PL_A2;
	char* callback  = (char*)PL_A3;
	long id = MongoInterface::get().watch(PL_A1, coll_name, callback, PL_A4);
	PL_A5 = PlTerm(id);
	return TRUE;
}

PREDICATE(mng_unwatch, 1) {
	long watcher_id = (long)PL_A1;
	MongoInterface::get().unwatch(watcher_id);
	return TRUE;
}


PREDICATE(mng_cursor_create, 3) {
	PL_A3 = MongoInterface::get().cursor_create(PL_A1,(char*)PL_A2)->id().c_str();
	return TRUE;
}

PREDICATE(mng_cursor_create, 4) {
	auto cursor = MongoInterface::get().cursor_create(PL_A1,(char*)PL_A2);
	cursor->filter(PL_A4);
	PL_A3 = cursor->id().c_str();
	return TRUE;
}

PREDICATE(mng_cursor_destroy, 1) {
	char* cursor_id = (char*)PL_A1;
	MongoInterface::get().cursor_destroy(cursor_id);
	return TRUE;
}

PREDICATE(mng_cursor_erase, 1) {
	return MongoInterface::get().cursor((char*)PL_A1)->erase();
}

PREDICATE(mng_cursor_filter, 2) {
	MongoInterface::get().cursor((char*)PL_A1)->filter(PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_aggregate, 2) {
	MongoInterface::get().cursor((char*)PL_A1)->aggregate(PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_descending, 2) {
	MongoInterface::get().cursor((char*)PL_A1)->descending((char*)PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_ascending, 2) {
	MongoInterface::get().cursor((char*)PL_A1)->ascending((char*)PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_limit, 2) {
	MongoInterface::get().cursor((char*)PL_A1)->limit((int)PL_A2);
	return TRUE;
}

PREDICATE(mng_cursor_next_pairs, 2) {
	const bson_t *doc;
	if(MongoInterface::get().cursor((char*)PL_A1)->next(&doc)) {
		PL_A2 = bson_to_term(doc);
		return TRUE;
	}
	else {
		return FALSE;
	}
}

PREDICATE(mng_cursor_next_json, 2) {
	const bson_t *doc;
	if(MongoInterface::get().cursor((char*)PL_A1)->next(&doc)) {
		char* str = bson_as_canonical_extended_json(doc, nullptr);
		PL_A2 = str;
		bson_free(str);
		return TRUE;
	}
	else {
		return FALSE;
	}
}
