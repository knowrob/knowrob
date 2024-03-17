/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mongoc.h>
#include <sstream>

#include "knowrob/backend/mongo/MongoInterface.h"

using namespace knowrob::mongo;

MongoInterface::MongoInterface() {
	mongoc_init();
}

MongoInterface::~MongoInterface() = default;

MongoInterface &MongoInterface::get() {
	static MongoInterface singleton;
	return singleton;
}

std::shared_ptr<Connection> MongoInterface::getOrCreateConnection(const char *uri_string_c) {
	std::string uri_string(uri_string_c);
	auto it = connections_.find(uri_string);
	if (it == connections_.end()) {
		// create a new connection
		auto newConnection = std::make_shared<Connection>(uri_string);
		connections_[uri_string] = newConnection;
		return newConnection;
	} else {
		return it->second;
	}
}

std::shared_ptr<Database> MongoInterface::connect(const PlTerm &dbTerm) {
	auto dbConnection = getOrCreateConnection((char *) dbTerm[1]);
	return std::make_shared<Database>(dbConnection->pool_, (char *) dbTerm[2]);
}

std::shared_ptr<Collection> MongoInterface::connect(const PlTerm &dbTerm, const char *collectionName) {
	return connect((char *) dbTerm[1], (char *) dbTerm[2], collectionName);
}

std::shared_ptr<Collection>
MongoInterface::connect(const char *db_uri, const char *db_name, const char *collectionName) {
	auto dbConnection = getOrCreateConnection(db_uri);
	return std::make_shared<Collection>(dbConnection, db_name, collectionName);
}


std::shared_ptr<Cursor> MongoInterface::cursor_create(const PlTerm &db_term, const char *coll_name) {
	auto coll = connect(db_term, coll_name);
	auto c = std::make_shared<Cursor>(coll);
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	cursors_[c->id()] = c;
	return c;
}

void MongoInterface::cursor_destroy(const char *curser_id) {
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	cursors_.erase(curser_id);
}

std::shared_ptr<Cursor> MongoInterface::cursor(const char *curser_id) {
	std::lock_guard<std::mutex> scoped_lock(mongo_mutex_);
	return cursors_[std::string(curser_id)];
}

