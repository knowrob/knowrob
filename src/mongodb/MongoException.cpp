#include "knowrob/mongodb/MongoException.h"

MongoException::MongoException(
	const char *scope,
	const bson_error_t &bson_err)
: PlException(PlCompound("mng_error",
	PlCompound(scope,
	PlTerm(bson_err.message))))
{}
