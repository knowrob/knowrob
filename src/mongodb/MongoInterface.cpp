/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mongoc.h>
#include <sstream>

#include <knowrob/mongodb/MongoInterface.h>

MongoInterface::MongoInterface()
{
	mongoc_init();
}
MongoInterface::~MongoInterface() = default;

MongoInterface::Connection::Connection(const std::string &uri_string)
: uri_string_(uri_string)
{
	bson_error_t err;
	uri_ = mongoc_uri_new_with_error(uri_string_.c_str(),&err);
	if(!uri_) {
		throw MongoException("invalid_uri",err);
	}
	pool_ = mongoc_client_pool_new(uri_);
	watch_ = std::make_shared<MongoWatch>(pool_);
	mongoc_client_pool_set_error_api(pool_, 2);
}

MongoInterface::Connection::~Connection()
{
	mongoc_client_pool_destroy(pool_);
	mongoc_uri_destroy(uri_);
	mongoc_cleanup();
}

MongoInterface& MongoInterface::get()
{
	static MongoInterface singleton;
	return singleton;
}

std::shared_ptr<MongoInterface::Connection> MongoInterface::getOrCreateConnection(const char *uri_string_c)
{
	std::string uri_string(uri_string_c);
	auto it = connections_.find(uri_string);
	if(it == connections_.end()) {
		// create a new connection
		auto newConnection = std::make_shared<MongoInterface::Connection>(uri_string);
		connections_[uri_string] = newConnection;
		return newConnection;
	}
	else {
		return it->second;
	}
}

long MongoInterface::watch(const PlTerm &db_term,
						   const char *coll_name,
						   const std::string &callback_goal,
						   const PlTerm &query_term)
{
	auto connection = getOrCreateConnection((char*)db_term[1]);
	auto watch_id = connection->watch_->watch(
			(char*)db_term[2],
			coll_name, callback_goal, query_term);
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	watcher_[watch_id] = connection;
	return watch_id;
}

void MongoInterface::unwatch(long watch_id)
{
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	auto it = watcher_.find(watch_id);
	if(it != watcher_.end()) {
		it->second->watch_->unwatch(watch_id);
		watcher_.erase(it);
	}
}

std::shared_ptr<MongoDatabase> MongoInterface::connect(const PlTerm &dbTerm)
{
	auto dbConnection = getOrCreateConnection((char*)dbTerm[1]);
	return std::make_shared<MongoDatabase>(dbConnection->pool_, (char*)dbTerm[2]);
}

std::shared_ptr<MongoCollection> MongoInterface::connect(const PlTerm &dbTerm, const char* collectionName)
{
	auto dbConnection = getOrCreateConnection((char*)dbTerm[1]);
	return std::make_shared<MongoCollection>(dbConnection->pool_, (char*)dbTerm[2], collectionName);
}

std::shared_ptr<MongoCursor> MongoInterface::cursor_create(const PlTerm &db_term, const char *coll_name)
{
	auto coll = connect(db_term, coll_name);
	auto c = std::make_shared<MongoCursor>(coll);
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	cursors_[c->id()] = c;
	return c;
}

void MongoInterface::cursor_destroy(const char *curser_id)
{
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	cursors_.erase(curser_id);
}

std::shared_ptr<MongoCursor> MongoInterface::cursor(const char *curser_id)
{
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	return cursors_[std::string(curser_id)];
}
