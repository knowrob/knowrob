#ifndef KNOWROB_MONGO_EXCEPTION_H
#define KNOWROB_MONGO_EXCEPTION_H

#include <mongoc.h>
#include <fmt/core.h>

namespace knowrob {
    /**
     * A runtime exception that occurred when interacting with Mongo DB.
     */
    class MongoException : public std::runtime_error {
    public:
        MongoException(const char *msg, const bson_error_t &err)
                : std::runtime_error(fmt::format("mng_error({},{})", msg, err.message)) {}
    };
}

#endif //KNOWROB_MONGO_EXCEPTION_H
