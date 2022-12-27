/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/mongodb/MongoDatabase.h"

MongoDatabase::MongoDatabase(
	    mongoc_client_pool_t *pool,
	    const char *db_name)
: pool_(pool)
{
	client_ = mongoc_client_pool_pop(pool_);
	db_ = mongoc_client_get_database(client_, db_name);
}

MongoDatabase::~MongoDatabase()
{
	mongoc_database_destroy(db_);
	mongoc_client_pool_push(pool_, client_);
}

mongoc_database_t* MongoDatabase::operator()()
{
	return db_;
}
