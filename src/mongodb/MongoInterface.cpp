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

using namespace knowrob::mongo;

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
		throw MongoException("invalid_uri", err);
	}
	pool_ = mongoc_client_pool_new(uri_);
    connectionWatch_ = std::make_shared<QueryWatch>(pool_);
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
                           const bson_t *query,
                           const ChangeStreamCallback &callback)
{
	auto connection = getOrCreateConnection((char*)db_term[1]);
	auto watch_id = connection->connectionWatch_->watch(
			(char*)db_term[2],
			coll_name, query, callback);
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	watcher_[watch_id] = connection;
	return watch_id;
}

void MongoInterface::unwatch(long watch_id)
{
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	auto it = watcher_.find(watch_id);
	if(it != watcher_.end()) {
		it->second->connectionWatch_->unwatch(watch_id);
		watcher_.erase(it);
	}
}

std::shared_ptr<Database> MongoInterface::connect(const PlTerm &dbTerm)
{
	auto dbConnection = getOrCreateConnection((char*)dbTerm[1]);
	return std::make_shared<Database>(dbConnection->pool_, (char*)dbTerm[2]);
}

std::shared_ptr<Collection> MongoInterface::connect(const PlTerm &dbTerm, const char* collectionName)
{
	return connect((char*)dbTerm[1], (char*)dbTerm[2], collectionName);
}

std::shared_ptr<Collection> MongoInterface::connect(const char *db_uri, const char *db_name, const char *collectionName)
{
    auto dbConnection = getOrCreateConnection(db_uri);
    return std::make_shared<Collection>(dbConnection->pool_, db_name, collectionName);
}


std::shared_ptr<Cursor> MongoInterface::cursor_create(const PlTerm &db_term, const char *coll_name)
{
	auto coll = connect(db_term, coll_name);
	auto c = std::make_shared<Cursor>(coll);
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	cursors_[c->id()] = c;
	return c;
}

void MongoInterface::cursor_destroy(const char *curser_id)
{
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	cursors_.erase(curser_id);
}

std::shared_ptr<Cursor> MongoInterface::cursor(const char *curser_id)
{
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	return cursors_[std::string(curser_id)];
}

