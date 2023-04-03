//
// Created by daniel on 01.04.23.
//

#include "knowrob/mongodb/MongoBulkOperation.h"
#include "knowrob/mongodb/MongoException.h"

using namespace knowrob;

MongoBulkOperation::MongoBulkOperation(mongoc_bulk_operation_t *handle)
: handle_(handle)
{
}

MongoBulkOperation::~MongoBulkOperation()
{
    if(handle_) {
        mongoc_bulk_operation_destroy(handle_);
        handle_ = nullptr;
    }
}

void MongoBulkOperation::validateBulkHandle()
{
    if(!handle_) {
        bson_error_t err;
        bson_set_error(&err,
                       MONGOC_ERROR_COMMAND,
                       MONGOC_ERROR_COMMAND_INVALID_ARG,
                       "bulk operation can only be executed once");
        throw MongoException("bulk_operation", err);
    }
}

void MongoBulkOperation::pushInsert(bson_t *document)
{
    validateBulkHandle();

    bson_error_t err;
    if(!mongoc_bulk_operation_insert_with_opts(
            handle_,
            document,
            nullptr,
            &err))
    {
        throw MongoException("bulk_operation",err);
    }
}

void MongoBulkOperation::pushRemoveOne(bson_t *document)
{
    validateBulkHandle();

    bson_error_t err;
    if(!mongoc_bulk_operation_remove_one_with_opts(
            handle_,
            document,
            nullptr,
            &err))
    {
        throw MongoException("bulk_operation",err);
    }
}

void MongoBulkOperation::pushRemoveAll(bson_t *document)
{
    validateBulkHandle();

    bson_error_t err;
    if(!mongoc_bulk_operation_remove_many_with_opts(
            handle_,
            document,
            nullptr,
            &err))
    {
        throw MongoException("bulk_operation",err);
    }
}

void MongoBulkOperation::pushUpdate(bson_t *query, bson_t *update)
{
    validateBulkHandle();

    bson_error_t err;
    if(!mongoc_bulk_operation_update_many_with_opts(
            handle_,
            query,
            update,
            nullptr,
            &err))
    {
        throw MongoException("bulk_operation",err);
    }
}


void MongoBulkOperation::execute()
{
    validateBulkHandle();

    bson_error_t bulk_err;
    bson_t bulk_reply;
    // perform the bulk write
    bool success = mongoc_bulk_operation_execute(handle_, &bulk_reply, &bulk_err);
    // cleanup
    bson_destroy(&bulk_reply);
    mongoc_bulk_operation_destroy(handle_);
    handle_ = nullptr;
    // throw exception on error
    if(!success) {
        throw MongoException("bulk_operation",bulk_err);
    }
}
