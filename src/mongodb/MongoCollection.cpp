/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/mongodb/MongoCollection.h>
#include <knowrob/mongodb/MongoException.h>
#include <knowrob/mongodb/MongoDocument.h>
#include <knowrob/mongodb/bson_pl.h>

using namespace knowrob;

static const mongoc_insert_flags_t INSERT_NO_VALIDATE_FLAG =
		(mongoc_insert_flags_t)MONGOC_INSERT_NO_VALIDATE;
static const mongoc_update_flags_t UPDATE_NO_VALIDATE_FLAG =
		(mongoc_update_flags_t)MONGOC_UPDATE_NO_VALIDATE;

MongoCollection::MongoCollection(
	    mongoc_client_pool_t *pool,
	    const std::string &db_name,
	    const std::string &coll_name)
: pool_(pool),
  name_(coll_name),
  dbName_(db_name),
  isSessionOwned_(true)
{
	client_ = mongoc_client_pool_pop(pool_);
	{
		bson_error_t error;
		session_ = mongoc_client_start_session (client_, nullptr, &error);
	}
	coll_ = mongoc_client_get_collection(client_, db_name.c_str(), coll_name.c_str());
    db_ = mongoc_client_get_database(client_, db_name.c_str());
}

MongoCollection::MongoCollection(
        mongoc_client_pool_t *pool,
        mongoc_client_t *client,
        mongoc_client_session_t *session,
        const std::string &db_name,
        const std::string &coll_name)
        : pool_(pool),
          client_(client),
          session_(session),
          name_(coll_name),
          dbName_(db_name),
          isSessionOwned_(false)
{
    coll_ = mongoc_client_get_collection(client_, db_name.c_str(), coll_name.c_str());
    db_ = mongoc_client_get_database(client_, db_name.c_str());
}

MongoCollection::~MongoCollection()
{
    mongoc_database_destroy(db_);
	mongoc_collection_destroy(coll_);
    if(isSessionOwned_) {
        if(session_) {
            mongoc_client_session_destroy(session_);
            session_ = nullptr;
        }
        mongoc_client_pool_push(pool_, client_);
    }
}

void MongoCollection::appendSession(bson_t *opts)
{
	if(session_!=nullptr) {
		bson_error_t error;
		if(!mongoc_client_session_append(session_, opts, &error)) {
			// TODO make use of KB_LOG macro
			//ROS_WARN("[MongoCollection] unable to append session to opts: %s.", error.message);
		}
	}
}

void MongoCollection::drop()
{
	bson_error_t err;
	if(!mongoc_collection_drop(coll_,&err)) {
		throw MongoException("drop_failed",err);
	}
}

void MongoCollection::storeOne(const MongoDocument &document)
{
    bson_error_t err;
    if(!mongoc_collection_insert(
            coll_,
            INSERT_NO_VALIDATE_FLAG,
            document.bson(),
            nullptr,
            &err))
    {
        throw MongoException("insert_failed",err);
    }
}

void MongoCollection::removeAll(const MongoDocument &document)
{
    remove(document, MONGOC_REMOVE_NONE);
}

void MongoCollection::removeOne(const MongoDocument &document)
{
    remove(document, MONGOC_REMOVE_SINGLE_REMOVE);
}

void MongoCollection::remove(const MongoDocument &document, mongoc_remove_flags_t flag)
{
    bson_error_t err;
    if(!mongoc_collection_remove(
            coll_,
            MONGOC_REMOVE_NONE,
            document.bson(),
            nullptr,
            &err))
    {
        throw MongoException("collection_remove",err);
    }
}


void MongoCollection::update(const MongoDocument &query, const MongoDocument &update)
{
    bson_error_t err;
    if(!mongoc_collection_update(
            coll_,
            UPDATE_NO_VALIDATE_FLAG,
            query.bson(),
            update.bson(),
            nullptr,
            &err))
    {
        throw MongoException("update_failed",err);
    }
}

void MongoCollection::evalAggregation(const MongoDocument &pipeline)
{
    auto cursor = mongoc_collection_aggregate(
            coll_,
            MONGOC_QUERY_NONE,
            pipeline.bson(),
            nullptr,
            nullptr);
    // make sure cursor has no error after creation
    bson_error_t err;
    if(mongoc_cursor_error(cursor, &err)) {
        mongoc_cursor_destroy(cursor);
        throw MongoException("cursor_error",err);
    }
    // process the query (ignore results)
    const bson_t *cursor_doc;
    while(mongoc_cursor_next(cursor,&cursor_doc)) {}
    mongoc_cursor_destroy(cursor);
}

std::shared_ptr<knowrob::MongoBulkOperation> MongoCollection::createBulkOperation()
{
    bson_t opts = BSON_INITIALIZER;
    BSON_APPEND_BOOL(&opts, "ordered", false);
    // create the bulk operation
    mongoc_bulk_operation_t *bulk =
            mongoc_collection_create_bulk_operation_with_opts(coll_, &opts);
    bson_destroy(&opts);
    return std::make_shared<knowrob::MongoBulkOperation>(bulk);
}

void MongoCollection::createIndex_internal(const bson_t &keys)
{
    bson_error_t err;
    bson_t reply;

    char *index_name = mongoc_collection_keys_to_index_string(&keys);
    bson_t *cmd = BCON_NEW ("createIndexes", BCON_UTF8(mongoc_collection_get_name(coll_)),
                            "indexes", "[", "{",
                            "key",  BCON_DOCUMENT(&keys),
                            "name", BCON_UTF8(index_name),
                            "}",  "]");
    bool success = mongoc_database_write_command_with_opts (
            db_,
            cmd,
            nullptr /* opts */,
            &reply,
            &err);

    bson_free(index_name);
    bson_destroy(&reply);
    bson_destroy(cmd);
    if(!success) {
        throw MongoException("create_index_failed",err);
    }
}

void MongoCollection::createAscendingIndex(const std::vector<const char*> &keys)
{
    bson_t b_keys;
    bson_init(&b_keys);
    for(auto key : keys) BSON_APPEND_INT32(&b_keys, key, 1);
    createIndex_internal(b_keys);
}

void MongoCollection::createIndex(const std::vector<MongoIndexKey> &keys)
{
    bson_t b_keys;
    bson_init(&b_keys);
    for(auto key : keys) {
        if(key.ascending) {
            BSON_APPEND_INT32(&b_keys, key.value, 1);
        }
        else {
            BSON_APPEND_INT32(&b_keys, key.value, -1);
        }
    }
    createIndex_internal(b_keys);
}
