/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KB_MONGO_DATABASE_H__
#define __KB_MONGO_DATABASE_H__

// STD
#include <memory>
#include <string>
// MongoDB
#include <mongoc.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**
 * A MongoDB database that can be accessed via a URI.
 */
class MongoDatabase {
public:
	MongoDatabase(mongoc_client_pool_t *pool, const std::string &db_name);
	~MongoDatabase();

	mongoc_database_t* db() { return db_; }

	bool create_index(const char *coll_name, const PlTerm &keys_pl);
	
private:
	mongoc_client_pool_t *pool_;
	mongoc_client_t *client_;
	mongoc_database_t *db_;
};

#endif //__KB_MONGO_DATABASE_H__
