/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/mongodb/MongoCollection.h>
#include <knowrob/mongodb/MongoException.h>
#include <knowrob/mongodb/bson_pl.h>

static const mongoc_insert_flags_t INSERT_NO_VALIDATE_FLAG =
		(mongoc_insert_flags_t)MONGOC_INSERT_NO_VALIDATE;
static const mongoc_update_flags_t UPDATE_NO_VALIDATE_FLAG =
		(mongoc_update_flags_t)MONGOC_UPDATE_NO_VALIDATE;

static const PlAtom ATOM_insert("insert");
static const PlAtom ATOM_remove("remove");
static const PlAtom ATOM_update("update");

MongoCollection::MongoCollection(
	    mongoc_client_pool_t *pool,
	    const std::string &db_name,
	    const std::string &coll_name)
: pool_(pool)
{
	client_ = mongoc_client_pool_pop(pool_);
	{
		bson_error_t error;
		session_ = mongoc_client_start_session (client_, nullptr, &error);
	}
	coll_ = mongoc_client_get_collection(client_, db_name.c_str(), coll_name.c_str());
}

MongoCollection::~MongoCollection()
{
	mongoc_collection_destroy(coll_);
	if(session_) {
		mongoc_client_session_destroy(session_);
		session_ = nullptr;
	}
	mongoc_client_pool_push(pool_, client_);
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

bool MongoCollection::drop()
{
	bson_error_t err;
	if(!mongoc_collection_drop(coll_,&err)) {
		throw MongoException("drop_failed",err);
	}
	else {
		return true;
	}
}

bool MongoCollection::store(const PlTerm &doc_term)
{
	bson_error_t err;
	bson_t *doc = bson_new();
	if(!bsonpl_concat(doc,doc_term,&err)) {
		bson_destroy(doc);
		throw MongoException("invalid_term",err);
	}
	bool success = mongoc_collection_insert(
			coll_,INSERT_NO_VALIDATE_FLAG,
			doc, nullptr,&err);
	bson_destroy(doc);
	if(!success) {
		throw MongoException("insert_failed",err);
	}
	return success;
}

bool MongoCollection::remove(const PlTerm &doc_term)
{
	bson_error_t err;
	bson_t *doc = bson_new();
	if(!bsonpl_concat(doc,doc_term,&err)) {
		bson_destroy(doc);
		throw MongoException("invalid_term",err);
	}
	bool success = mongoc_collection_remove(
			coll_,MONGOC_REMOVE_NONE,
			doc, nullptr,&err);
	bson_destroy(doc);
	if(!success) {
		throw MongoException("collection_remove",err);
	}
	return success;
}

bool MongoCollection::update(
		const PlTerm &query_term, const PlTerm &update_term)
{
	bson_error_t err;

	bson_t *query = bson_new();
	if(!bsonpl_concat(query,query_term,&err)) {
		bson_destroy(query);
		throw MongoException("invalid_query",err);
	}

	bson_t *update = bson_new();
	if(!bsonpl_concat(update,update_term,&err)) {
		bson_destroy(query);
		bson_destroy(update);
		throw MongoException("invalid_update",err);
	}

	bool success = mongoc_collection_update(
			coll_, UPDATE_NO_VALIDATE_FLAG,
			query, update, nullptr, &err);
	bson_destroy(query);
	bson_destroy(update);
	if(!success) {
		throw MongoException("update_failed",err);
	}
	return success;
}

bool MongoCollection::bulk_write(const PlTerm &doc_term)
{
	bson_t reply;
	// bulk options: set ordered to false to allow server performing
	//               the steps in parallel
	bson_t opts = BSON_INITIALIZER;
	BSON_APPEND_BOOL(&opts, "ordered", false);
	// create the bulk operation
	mongoc_bulk_operation_t *bulk =
			mongoc_collection_create_bulk_operation_with_opts(coll_, nullptr);
	// iterate over input list and insert steps
	PlTail pl_list(doc_term);
	PlTerm pl_member;
	while(pl_list.next(pl_member)) {
		const PlAtom operation_name(pl_member.name());
		const PlTerm &pl_value1 = pl_member[1];
		bool is_operation_queued = false;
		bson_error_t err;
		// parse the document
		bson_t *doc1 = bson_new();
		if(!bsonpl_concat(doc1,pl_value1,&err)) {
			bson_destroy(doc1);
			mongoc_bulk_operation_destroy(bulk);
			throw MongoException("invalid_term",err);
		}
		if(operation_name == ATOM_insert) {
			is_operation_queued = mongoc_bulk_operation_insert_with_opts(
					bulk, doc1, nullptr, &err);
		}
		else if(operation_name == ATOM_remove) {
			is_operation_queued = mongoc_bulk_operation_remove_many_with_opts(
					bulk, doc1, nullptr, &err);
		}
		else if(operation_name == ATOM_update) {
			const PlTerm &pl_value2 = pl_member[2];
			bson_t *doc2 = bson_new();
			if(!bsonpl_concat(doc2,pl_value2,&err)) {
				bson_destroy(doc1);
				bson_destroy(doc2);
				mongoc_bulk_operation_destroy(bulk);
				throw MongoException("invalid_term",err);
			}
			is_operation_queued = mongoc_bulk_operation_update_many_with_opts(
					bulk, doc1, doc2, nullptr, &err);
			bson_destroy(doc2);
		}
		else {
			bson_set_error(&err,
						   MONGOC_ERROR_COMMAND,
						   MONGOC_ERROR_COMMAND_INVALID_ARG,
						   "unknown bulk operation '%s'", pl_member.name());
		}
		bson_destroy(doc1);
		if(!is_operation_queued) {
			mongoc_bulk_operation_destroy(bulk);
			throw MongoException("bulk_operation",err);
		}
	}
	bson_error_t bulk_err;
	// perform the bulk write
	bool success = mongoc_bulk_operation_execute(bulk, &reply, &bulk_err);
	// cleanup
	bson_destroy(&reply);
	mongoc_bulk_operation_destroy(bulk);
	// throw exception on error
	if(!success) {
		throw MongoException("bulk_operation",bulk_err);
	}
	return success;
}
