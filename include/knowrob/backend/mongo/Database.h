/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_DATABASE_H
#define KNOWROB_MONGO_DATABASE_H

#include <memory>
#include <string>
#include <mongoc.h>
#include "Index.h"

namespace knowrob::mongo {
    /**
     * A named database in a Mongo DB instance.
     */
    class Database {
    public:
        Database(mongoc_client_pool_t *pool, std::string_view db_name);
        ~Database();

        mongoc_database_t* db() { return db_; }

        bool create_index(const char *coll_name, const std::vector<IndexKey> &indexes);

    private:
        mongoc_client_pool_t *pool_;
        mongoc_client_t *client_;
        mongoc_database_t *db_;
    };
}

#endif //KNOWROB_MONGO_DATABASE_H
