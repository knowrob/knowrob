/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_DATABASE_H
#define KNOWROB_MONGO_DATABASE_H

#include <memory>
#include <string>
#include <mongoc.h>
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

namespace knowrob::mongo {
    /**
     * A MongoDB database that can be accessed via a URI.
     */
    class Database {
    public:
        Database(mongoc_client_pool_t *pool, const std::string &db_name);
        ~Database();

        mongoc_database_t* db() { return db_; }

        bool create_index(const char *coll_name, const PlTerm &keys_pl);

    private:
        mongoc_client_pool_t *pool_;
        mongoc_client_t *client_;
        mongoc_database_t *db_;
    };
}

#endif //KNOWROB_MONGO_DATABASE_H
