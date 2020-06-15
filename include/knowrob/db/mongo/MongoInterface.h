/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KB_MONGO_IFACE_H__
#define __KB_MONGO_IFACE_H__

#include <mongoc.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <map>
#include <string>
#include <mutex>

#include <knowrob/db/mongo/MongoException.h>
#include <knowrob/db/mongo/MongoDatabase.h>
#include <knowrob/db/mongo/MongoCollection.h>
#include <knowrob/db/mongo/MongoCursor.h>

class MongoInterface {
public:
	static MongoInterface& get();
	
	static mongoc_client_pool_t* pool();
	
	
	void drop(const char *db_name, const char *coll_name);
	
	void store(const char *db_name, const char *coll_name, const PlTerm &doc_term);
	
	void update(const char *db_name, const char *coll_name, const PlTerm &query_term, const PlTerm &update_term);
	
	void remove(const char *db_name, const char *coll_name, const PlTerm &doc_term);
	
	void create_index(const char *db_name, const char *coll_name, const PlTerm &keys_term);
	
	
	static MongoCursor* cursor_create(const char *db_name, const char *coll_name);
	
	static void cursor_destroy(const char *curser_id);
	
	static MongoCursor* cursor(const char *curser_id);
	
private:
	MongoInterface();
	~MongoInterface();
	
	MongoInterface(MongoInterface const&); // Don't Implement
	void operator=(MongoInterface const&); // Don't implement
	
	mongoc_uri_t *uri_;
	mongoc_client_pool_t *pool_;
	
	std::map<std::string, MongoCursor*> cursors_;

	std::mutex mongo_mutex_;
};

#endif //__KB_MONGO_IFACE_H__
