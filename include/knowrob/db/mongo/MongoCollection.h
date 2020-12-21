/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KB_MONGO_COLLECTION_H__
#define __KB_MONGO_COLLECTION_H__

#include <mongoc.h>

class MongoCollection {
public:
	MongoCollection(
		mongoc_client_pool_t *pool,
		const char *db_name,
		const char *coll_name);
	~MongoCollection();
	
	void appendSession(bson_t *opts);

	mongoc_client_session_t* session() { return session_; }

	mongoc_collection_t* operator()();
	
private:
	mongoc_client_pool_t *pool_;
	mongoc_client_t *client_;
	mongoc_client_session_t *session_;
	mongoc_collection_t *coll_;
};

#endif //__KB_MONGO_COLLECTION_H__
