/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KB_MONGO_DATABASE_H__
#define __KB_MONGO_DATABASE_H__

#include <mongoc.h>

class MongoDatabase {
public:
	MongoDatabase(
		mongoc_client_pool_t *pool,
		const char *db_name);
	~MongoDatabase();
	
	mongoc_database_t* operator()();
	
private:
	mongoc_client_pool_t *pool_;
	mongoc_client_t *client_;
	mongoc_database_t *db_;
};

#endif //__KB_MONGO_COLLECTION_H__
