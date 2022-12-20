/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/MongoCollection.h"

MongoCollection::MongoCollection(
	    mongoc_client_pool_t *pool,
	    const char *db_name,
	    const char *coll_name)
: pool_(pool)
{
	client_ = mongoc_client_pool_pop(pool_);
	{
		bson_error_t error;
		session_ = mongoc_client_start_session (client_, NULL, &error);
	}
	coll_ = mongoc_client_get_collection(client_, db_name, coll_name);
}

MongoCollection::~MongoCollection()
{
	mongoc_collection_destroy(coll_);
	if(session_) {
		mongoc_client_session_destroy(session_);
		session_ = NULL;
	}
	mongoc_client_pool_push(pool_, client_);
}

void MongoCollection::appendSession(bson_t *opts)
{
	if(session_!=NULL) {
		bson_error_t error;
		if(!mongoc_client_session_append(session_, opts, &error)) {
			// TODO make use of KB_LOG macro
			//ROS_WARN("[MongoCollection] unable to append session to opts: %s.", error.message);
		}
	}
}

mongoc_collection_t* MongoCollection::operator()()
{
	return coll_;
}
