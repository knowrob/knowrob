/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_CONNECTION_H
#define KNOWROB_MONGO_CONNECTION_H

#include <string>
#include <mongoc/mongoc.h>

namespace knowrob::mongo {
	/**
	 * A connection to a Mongo DB.
	 */
    struct Connection {
        explicit Connection(std::string_view uri_string);

        ~Connection();

        /**
         * The URI of the connection.
		 */
        mongoc_uri_t *uri_;
		/**
		 * The URI string.
		 */
        std::string uri_string_;
		/**
		 * The connection pool.
		 */
        mongoc_client_pool_t *pool_;
    };
}


#endif //KNOWROB_MONGO_CONNECTION_H
