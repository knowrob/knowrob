#ifndef __KB_BSON_MONGO_H__
#define __KB_BSON_MONGO_H__

#include <mongoc.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

bson_t* bson_new_from_term(const PlTerm &term, bson_error_t *err);

PlTerm bson_to_term(const bson_t *bson);

#endif //__KB_BSON_MONGO_H__
