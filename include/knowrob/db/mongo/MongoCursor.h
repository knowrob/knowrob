/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KB_MONGO_CURSOR_H__
#define __KB_MONGO_CURSOR_H__

#include <mongoc.h>

#include <string>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>
// knowrob_mongo
#include <knowrob/db/mongo/MongoCollection.h>

class MongoCursor {
public:
	MongoCursor(
		mongoc_client_pool_t *pool,
		const char *db_name,
		const char *coll_name);
	~MongoCursor();
	
	const std::string& id() { return id_; };
	
	void limit(unsigned int limit);
	
	void ascending(const char *key);
	
	void descending(const char *key);
	
	void filter(const PlTerm &query_term);
	
	void aggregate(const PlTerm &query_term);

	bool next(const bson_t **doc);
	
	bool erase();
	
private:
	MongoCollection coll_;
	mongoc_cursor_t *cursor_;
	bson_t *query_;
	bson_t *opts_;
	std::string id_;
	bool is_aggregate_query_;
};

#endif //__KB_MONGO_CURSOR_H__
