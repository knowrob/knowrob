/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>
// STD
#include <iostream>
// KnowRob
#include "knowrob/backend/mongo/MongoInterface.h"
#include "knowrob/reasoner/mongolog/bson_pl.h"
#include "knowrob/backend/mongo/Document.h"

using namespace knowrob;
using namespace knowrob::mongo;

#define PREDICATE_COLLECTION MongoInterface::get().connect(PL_A1, (char*)PL_A2)
#define PREDICATE_CURSOR MongoInterface::get().cursor((char*)PL_A1)

namespace knowrob {
	class MongoPLException : public PlException, std::exception {
	public:
		explicit MongoPLException(const MongoException &exc)
				: PlException(PlCompound(
				"mng_error", PlCompound(exc.contextMessage_.c_str(), PlTerm(exc.bsonMessage_.c_str()))
		)) {}

		explicit MongoPLException(const std::exception &exc)
				: PlException(PlCompound("mng_error", PlTerm(exc.what()))) {}
	};
}

static inline bson_t* termToDocument(const PlTerm &term) {
    bson_error_t err;
    auto document = bson_new();
    if(!bsonpl_concat(document,term, &err)) {
        bson_free(document);
        throw MongoPLException(MongoException("invalid_term", err));
    }
    return document;
}

PREDICATE(mng_collections,2) {
	auto db_handle = MongoInterface::get().connect(PL_A1);
	bson_error_t err;
	char **strv;
	if ((strv = mongoc_database_get_collection_names_with_opts(
			db_handle->db(), nullptr /* opts */, &err))) {
		PlTail l(PL_A2);
		for (int i=0; strv[i]; i++) {
			l.append( strv[i] );
		}
		l.close();
		bson_strfreev(strv);
		return TRUE;
	}
	else {
		throw MongoPLException(MongoException("collection_lookup_failed", err));
	}
}

PREDICATE(mng_distinct_values_json,4) {
	auto db_handle = MongoInterface::get().connect(PL_A1);
	char* coll_name = (char*)PL_A2;
	char* key = (char*)PL_A3;
	bson_error_t err;
	bson_t reply;
	//
	bson_t *command = BCON_NEW("distinct", BCON_UTF8(coll_name), "key", BCON_UTF8(key));
	bool success = mongoc_database_command_simple(
			db_handle->db(), command, nullptr, &reply, &err);
	if(success) {
		char* str = bson_as_canonical_extended_json(&reply, nullptr);
		PL_A4 = str;
		bson_free(str);
	}
	bson_destroy(command);
	return success;
}

PREDICATE(mng_index_create_core, 3) {
	static const PlAtom ATOM_minus("-");

	std::vector<mongo::IndexKey> indexes;
	PlTail pl_list(PL_A3);
	PlTerm pl_member;
	while (pl_list.next(pl_member)) {
		const PlAtom mode_atom(pl_member.name());
		const PlTerm &pl_value = pl_member[1];
		if (mode_atom == ATOM_minus) {
			indexes.emplace_back((char *) pl_value, mongo::IndexType::DESCENDING);
		} else {
			indexes.emplace_back((char *) pl_value, mongo::IndexType::ASCENDING);
		}
	}

	return MongoInterface::get().connect(PL_A1)->create_index((char*)PL_A2,indexes);
}


PREDICATE(mng_drop_unsafe, 2) {
    try {
        PREDICATE_COLLECTION->drop();
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_store, 3) {
    try {
        PREDICATE_COLLECTION->storeOne(Document(termToDocument(PL_A3)));
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_remove, 3) {
    try {
        PREDICATE_COLLECTION->removeAll(Document(termToDocument(PL_A3)));
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_update, 4) {
    try {
        PREDICATE_COLLECTION->update(
                Document(termToDocument(PL_A3)),
                Document(termToDocument(PL_A4)));
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

/*
 * bool MongoCollection::bulk_write(const PlTerm &doc_term)
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
 */

PREDICATE(mng_bulk_write, 3) {
    static const PlAtom ATOM_insert("insert");
    static const PlAtom ATOM_remove("remove");
    static const PlAtom ATOM_update("update");

    try {
        auto bulk = PREDICATE_COLLECTION->createBulkOperation();
        PlTail pl_list(PL_A3);
        PlTerm pl_member;

        while(pl_list.next(pl_member)) {
            const PlAtom operation_name(pl_member.name());
            const auto &pl_value1 = pl_member[1];
            bson_error_t err;

            // parse the document
            auto doc1 = Document(bson_new());
            if(!bsonpl_concat(doc1.bson(),pl_value1,&err)) {
                throw MongoException("invalid_term", err);
            }

            if(operation_name == ATOM_insert) {
                bulk->pushInsert(doc1.bson());
            }
            else if(operation_name == ATOM_remove) {
                bulk->pushRemoveAll(doc1.bson());
            }
            else if(operation_name == ATOM_update) {
                const auto &pl_value2 = pl_member[2];
                auto doc2 = Document(bson_new());
                if(!bsonpl_concat(doc2.bson(), pl_value2, &err)) {
                    throw MongoException("invalid_term", err);
                }
                bulk->pushUpdate(doc1.bson(), doc2.bson());
            }
            else {
                bson_set_error(&err,
                               MONGOC_ERROR_COMMAND,
                               MONGOC_ERROR_COMMAND_INVALID_ARG,
                               "unknown bulk operation '%s'", pl_member.name());
                throw MongoException("bulk_error", err);
            }
        }

        bulk->execute();
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}



PREDICATE(mng_cursor_create, 3) {
    try {
        PL_A3 = MongoInterface::get().cursor_create(PL_A1,(char*)PL_A2)->id().c_str();
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_create, 4) {
    try {
        Document doc_a(termToDocument(PL_A4));
        auto cursor = MongoInterface::get().cursor_create(PL_A1,(char*)PL_A2);
        cursor->filter(doc_a.bson());
        PL_A3 = cursor->id().c_str();
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_destroy, 1) {
    try {
        char* cursor_id = (char*)PL_A1;
        MongoInterface::get().cursor_destroy(cursor_id);
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_erase, 1) {
    try {
	    return PREDICATE_CURSOR->erase();
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_filter, 2) {
    try {
        Document doc_a(termToDocument(PL_A2));
        PREDICATE_CURSOR->filter(doc_a.bson());
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_aggregate, 2) {
    try {
        Document doc_a(termToDocument(PL_A2));
        PREDICATE_CURSOR->aggregate(doc_a.bson());
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_descending, 2) {
    try {
        PREDICATE_CURSOR->descending((char*)PL_A2);
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_ascending, 2) {
    try {
        PREDICATE_CURSOR->ascending((char*)PL_A2);
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_limit, 2) {
    try {
        PREDICATE_CURSOR->limit((int)PL_A2);
        return TRUE;
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_next_pairs, 2) {
    try {
        const bson_t *doc;
        if(PREDICATE_CURSOR->next(&doc)) {
            PL_A2 = bson_to_term(doc);
            return TRUE;
        }
        else {
            return FALSE;
        }
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}

PREDICATE(mng_cursor_next_json, 2) {
    try {
        const bson_t *doc;
        if(PREDICATE_CURSOR->next(&doc)) {
            char* str = bson_as_canonical_extended_json(doc, nullptr);
            PL_A2 = str;
            bson_free(str);
            return TRUE;
        }
        else {
            return FALSE;
        }
    }
    catch(const MongoException &exc) { throw MongoPLException(exc); }
    catch(const std::exception &exc) { throw MongoPLException(exc); }
}
