/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mongoc.h>
#include <sstream>

#include <rosprolog/rosprolog_kb/rosprolog_kb.h>

#include "knowrob/db/mongo/MongoInterface.h"
#include "knowrob/db/mongo/bson_pl.h"

/*********************************/
/********** static functions *****/
/*********************************/

MongoInterface& MongoInterface::get()
{
	static MongoInterface the_iface;
	return the_iface;
}

mongoc_client_pool_t* MongoInterface::pool()
{
	return MongoInterface::get().pool_;
}

MongoCursor* MongoInterface::cursor_create(const char *db_name, const char *coll_name)
{
	MongoCursor *c = new MongoCursor(MongoInterface::pool(),db_name,coll_name);
	{
		std::lock_guard<std::mutex> scoped_lock(MongoInterface::get().mongo_mutex_);
		MongoInterface::get().cursors_[c->id()] = c;
	}
	return c;
}

void MongoInterface::cursor_destroy(const char *curser_id)
{
	std::lock_guard<std::mutex> scoped_lock(MongoInterface::get().mongo_mutex_);
	std::string key(curser_id);
	MongoCursor *c = MongoInterface::get().cursors_[key];
	MongoInterface::get().cursors_.erase(key);
	delete c;
}

MongoCursor* MongoInterface::cursor(const char *curser_id)
{
	MongoCursor *c;
	{
		std::lock_guard<std::mutex> scoped_lock(MongoInterface::get().mongo_mutex_);
		c = MongoInterface::get().cursors_[std::string(curser_id)];
	}
	return c;
}

/*********************************/
/********** MongoInterface *******/
/*********************************/

MongoInterface::MongoInterface()
{
	mongoc_init();
	bson_error_t err;
	std::string mongo_uri;
	if(!rosprolog_kb::node().getParam(std::string("mongodb_uri"),mongo_uri)) {
		char *mongo_host = std::getenv("MONGO_PORT_27017_TCP_ADDR");
		if(mongo_host != NULL) {
			std::stringstream ss;
			ss << "mongodb://" << mongo_host << ":27017/?appname=knowrob";
			mongo_uri = ss.str();
		}
		else {
			mongo_uri = std::string("mongodb://localhost:27017/?appname=knowrob");
		}
	}
	uri_ = mongoc_uri_new_with_error(mongo_uri.c_str(),&err);
	if(!uri_) {
		throw MongoException("invalid_uri",err);
	}
	pool_ = mongoc_client_pool_new(uri_);
	mongoc_client_pool_set_error_api(pool_, 2);
}

MongoInterface::~MongoInterface()
{
	mongoc_client_pool_destroy(pool_);
	mongoc_uri_destroy(uri_);
	mongoc_cleanup();
}

void MongoInterface::drop(const char *db_name, const char *coll_name)
{
	MongoCollection coll(pool_,db_name,coll_name);
	bson_error_t err;
	if(!mongoc_collection_drop(coll(),&err)) {
		throw MongoException("drop_failed",err);
	}
}

void MongoInterface::store(
		const char *db_name,
		const char *coll_name,
		const PlTerm &doc_term)
{
	MongoCollection coll(pool_,db_name,coll_name);
	bson_error_t err;
	//
	bson_t *doc = bson_new_from_term(doc_term,&err);
	if(doc==NULL) {
		throw MongoException("invalid_term",err);
	}
	bool success = mongoc_collection_insert(coll(),MONGOC_INSERT_NONE,doc,NULL,&err);
	bson_destroy(doc);
	if(!success) {
		throw MongoException("insert_failed",err);
	}
}

void MongoInterface::remove(
		const char *db_name,
		const char *coll_name,
		const PlTerm &doc_term)
{
	MongoCollection coll(pool_,db_name,coll_name);
	bson_error_t err;
	//
	bson_t *doc = bson_new_from_term(doc_term,&err);
	if(doc==NULL) {
		throw MongoException("invalid_term",err);
	}
	bool success = mongoc_collection_remove(coll(),MONGOC_REMOVE_NONE,doc,NULL,&err);
	bson_destroy(doc);
	if(!success) {
		throw MongoException("remove_failed",err);
	}
}

void MongoInterface::update(
		const char *db_name,
		const char *coll_name,
		const PlTerm &query_term,
		const PlTerm &update_term)
{
	MongoCollection coll(pool_,db_name,coll_name);
	bson_error_t err;
	//
	bson_t *query = bson_new_from_term(query_term, &err);
	if(query==NULL) {
		throw MongoException("invalid_query",err);
	}
	bson_t *update = bson_new_from_term(update_term, &err);
	if(update==NULL) {
		bson_destroy(query);
		throw MongoException("invalid_update",err);
	}
	bool success = mongoc_collection_update(coll(),
		MONGOC_UPDATE_MULTI_UPDATE,
		query,
		update,
		NULL,
		&err);
	bson_destroy(query);
	bson_destroy(update);
	if(!success) {
		throw MongoException("update_failed",err);
	}
}

void MongoInterface::create_index(const char *db_name, const char *coll_name, const PlTerm &keys_pl)
{
	MongoDatabase db_handle(pool_,db_name);
	bson_t reply;
	bson_error_t err;
	bson_t keys;
	//
	bson_init(&keys);
	PlTail pl_list(keys_pl);
	PlTerm pl_member;
	while(pl_list.next(pl_member)) {
		BSON_APPEND_INT32(&keys, (char*)pl_member, 1);
	}
	char *index_name = mongoc_collection_keys_to_index_string(&keys);
	//
	bson_t *cmd = BCON_NEW ("createIndexes", BCON_UTF8(coll_name),
				 "indexes", "[", "{",
						 "key",  BCON_DOCUMENT(&keys),
						 "name", BCON_UTF8(index_name),
					"}",  "]"
	);
	bool success = mongoc_database_write_command_with_opts (
				db_handle(), cmd, NULL /* opts */, &reply, &err);
	bson_free(index_name);
	bson_destroy(&reply);
	bson_destroy(cmd);
	if(!success) {
		throw MongoException("create_index_failed",err);
	}
}
