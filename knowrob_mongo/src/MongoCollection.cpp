/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob_mongo/MongoCollection.h"

MongoCollection::MongoCollection(
	    mongoc_client_pool_t *pool,
	    const char *db_name,
	    const char *coll_name)
: pool_(pool)
{
	client_ = mongoc_client_pool_pop(pool_);
	coll_ = mongoc_client_get_collection(client_, db_name, coll_name);
}

MongoCollection::~MongoCollection()
{
	mongoc_collection_destroy(coll_);
	mongoc_client_pool_push(pool_, client_);
}

mongoc_collection_t* MongoCollection::operator()()
{
	return coll_;
}
