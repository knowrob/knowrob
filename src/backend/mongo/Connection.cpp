/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/mongo/Connection.h"
#include "knowrob/backend/mongo/MongoException.h"
#include <string>

using namespace knowrob::mongo;

Connection::Connection(std::string_view uri_string)
		: uri_string_(uri_string) {
	bson_error_t err;
	uri_ = mongoc_uri_new_with_error(uri_string_.c_str(), &err);
	if (!uri_) {
		throw MongoException("invalid_uri", err);
	}
	pool_ = mongoc_client_pool_new(uri_);
	// connectionWatch_ = std::make_shared<QueryWatch>(pool_);
	mongoc_client_pool_set_error_api(pool_, 2);
}

Connection::~Connection() {
	mongoc_client_pool_destroy(pool_);
	mongoc_uri_destroy(uri_);
	mongoc_cleanup();
}
