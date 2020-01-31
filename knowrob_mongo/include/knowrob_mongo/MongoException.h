#ifndef __KB_MONGO_EXCEPTION_H__
#define __KB_MONGO_EXCEPTION_H__

#include <mongoc.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

class MongoException : public PlException
{
public:
	MongoException(const char *msg, const bson_error_t &err);
}; 

#endif //__KB_MONGO_EXCEPTION_H__
