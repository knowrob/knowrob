/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_INTERFACE_H
#define KNOWROB_MONGO_INTERFACE_H

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

#include <knowrob/backend/mongo/MongoException.h>
#include <knowrob/backend/mongo/Database.h>
#include <knowrob/backend/mongo/Collection.h>
#include <knowrob/backend/mongo/Cursor.h>
#include <knowrob/backend/mongo/QueryWatch.h>
#include "Connection.h"

namespace knowrob::mongo {
    class MongoInterface {
    public:
        static MongoInterface& get();

        std::shared_ptr<Database> connect(const PlTerm &dbTerm);

        std::shared_ptr<Collection> connect(const PlTerm &dbTerm, const char* collectionName);

        std::shared_ptr<Collection> connect(const char* db_uri, const char* db_name, const char* collectionName);

        std::shared_ptr<Cursor> cursor_create(const PlTerm &db_term, const char *coll_name);

        void cursor_destroy(const char *curser_id);

        std::shared_ptr<Cursor> cursor(const char *curser_id);




    private:
        MongoInterface();
        ~MongoInterface();

        MongoInterface(MongoInterface const&); // Don't Implement
        void operator=(MongoInterface const&); // Don't implements

        std::shared_ptr<Connection> getOrCreateConnection(const char *uri_string_c);

        // maps URI to MongoDatabase for all previously accessed MongoDatabase's
        std::map<std::string, std::shared_ptr<Connection>> connections_;
        std::map<std::string, std::shared_ptr<Cursor>> cursors_;
        std::map<long, std::shared_ptr<Connection>> watcher_;

        std::mutex mongo_mutex_;
    };
}

#endif //KNOWROB_MONGO_INTERFACE_H
