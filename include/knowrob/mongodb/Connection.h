//
// Created by sascha on 24.07.23.
//

#ifndef KNOWROB_MONGO_CONNECTION_H
#define KNOWROB_MONGO_CONNECTION_H

#include <string>
#include <mongoc/mongoc.h>
// #include "knowrob/mongodb/QueryWatch.h"

namespace knowrob::mongo {
    class Connection {
    public:
        mongoc_uri_t *uri_;
        mongoc_client_pool_t *pool_;
        // std::shared_ptr <QueryWatch> connectionWatch_;
        std::string uri_string_;

        explicit Connection(const std::string &uri_string);

        ~Connection();
    };


}


#endif //KNOWROB_MONGO_CONNECTION_H
