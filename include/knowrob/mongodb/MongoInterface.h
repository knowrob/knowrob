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
// STD
#include <map>
#include <memory>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <map>
#include <string>
#include <mutex>

#include <knowrob/mongodb/MongoException.h>
#include <knowrob/mongodb/MongoDatabase.h>
#include <knowrob/mongodb/MongoCollection.h>
#include <knowrob/mongodb/MongoCursor.h>
#include <knowrob/mongodb/MongoWatch.h>

class MongoInterface {
public:
	static MongoInterface& get();

	std::shared_ptr<MongoDatabase> connect(const PlTerm &dbTerm);

	std::shared_ptr<MongoCollection> connect(const PlTerm &dbTerm, const char* collectionName);

    std::shared_ptr<MongoCollection> connect(const char* db_uri, const char* db_name, const char* collectionName);
	
	std::shared_ptr<MongoCursor> cursor_create(const PlTerm &db_term, const char *coll_name);

	void cursor_destroy(const char *curser_id);
	
	std::shared_ptr<MongoCursor> cursor(const char *curser_id);

	long watch(const PlTerm &db_term,
			   const char *coll_name,
			   const std::string &callback_goal,
			   const PlTerm &query_term);

	void unwatch(long i);

private:
	MongoInterface();
	~MongoInterface();
	
	MongoInterface(MongoInterface const&); // Don't Implement
	void operator=(MongoInterface const&); // Don't implements

	class Connection {
	public:
		mongoc_uri_t *uri_;
		mongoc_client_pool_t *pool_;
		std::shared_ptr<MongoWatch> watch_;
		std::string uri_string_;

		explicit Connection(const std::string &uri_string);
		~Connection();
	};

	std::shared_ptr<MongoInterface::Connection> getOrCreateConnection(const char *uri_string_c);

	// maps URI to MongoDatabase for all previously accessed MongoDatabase's
	std::map<std::string, std::shared_ptr<Connection>> connections_;
	std::map<std::string, std::shared_ptr<MongoCursor>> cursors_;
	std::map<long, std::shared_ptr<Connection>> watcher_;

	std::mutex mongo_mutex_;
};

#endif //__KB_MONGO_IFACE_H__
