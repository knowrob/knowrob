#ifndef __KB_BSON_MONGO_H__
#define __KB_BSON_MONGO_H__

#include <mongoc.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

PlTerm bson_to_term(const bson_t *bson);

bool bsonpl_append(bson_t *doc, const char *key, const PlTerm &term, bson_error_t *err);

bool bsonpl_concat(bson_t *doc, const PlTerm &term, bson_error_t *err);

#endif //__KB_BSON_MONGO_H__
