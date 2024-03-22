/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/backend/mongo/Database.h>
#include <knowrob/backend/mongo/MongoException.h>

using namespace knowrob::mongo;

Database::Database(mongoc_client_pool_t *pool, std::string_view db_name)
		: pool_(pool) {
	client_ = mongoc_client_pool_pop(pool_);
	db_ = mongoc_client_get_database(client_, db_name.data());
}

Database::~Database() {
	mongoc_database_destroy(db_);
	mongoc_client_pool_push(pool_, client_);
}

bool Database::create_index(const char *coll_name, const std::vector<IndexKey> &indexes) {
	bson_t reply;
	bson_error_t err;
	bson_t keys;
	//
	bson_init(&keys);
	for (auto &is : indexes) {
		BSON_APPEND_INT32(&keys, is.value.c_str(), (int)is.type);
	}
	char *index_name = mongoc_collection_keys_to_index_string(&keys);
	//
	bson_t *cmd = BCON_NEW ("createIndexes", BCON_UTF8(coll_name),
							"indexes", "[", "{",
							"key", BCON_DOCUMENT(&keys),
							"name", BCON_UTF8(index_name),
							"}", "]"
	);
	bool success = mongoc_database_write_command_with_opts(
			db_, cmd, nullptr /* opts */, &reply, &err);
	bson_free(index_name);
	bson_destroy(&reply);
	bson_destroy(cmd);
	if (!success) {
		throw MongoException("create_index_failed", err);
	}
	return success;
}
