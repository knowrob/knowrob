/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/backend/mongo/Database.h>
#include <knowrob/backend/mongo/MongoException.h>

using namespace knowrob::mongo;

Database::Database(mongoc_client_pool_t *pool, const std::string &db_name)
		: pool_(pool) {
	client_ = mongoc_client_pool_pop(pool_);
	db_ = mongoc_client_get_database(client_, db_name.c_str());
}

Database::~Database() {
	mongoc_database_destroy(db_);
	mongoc_client_pool_push(pool_, client_);
}

bool Database::create_index(const char *coll_name, const PlTerm &keys_pl) {
	static const PlAtom ATOM_minus("-");

	bson_t reply;
	bson_error_t err;
	bson_t keys;
	//
	bson_init(&keys);
	PlTail pl_list(keys_pl);
	PlTerm pl_member;
	while (pl_list.next(pl_member)) {
		const PlAtom mode_atom(pl_member.name());
		const PlTerm &pl_value = pl_member[1];
		if (mode_atom == ATOM_minus) {
			BSON_APPEND_INT32(&keys, (char *) pl_value, -1);
		} else {
			BSON_APPEND_INT32(&keys, (char *) pl_value, 1);
		}
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
