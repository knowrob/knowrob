#ifndef KNOWROB_MONGO_EXCEPTION_H
#define KNOWROB_MONGO_EXCEPTION_H

#include <mongoc.h>
#include <fmt/core.h>

namespace knowrob::mongo {
    /**
     * A runtime exception that occurred when interacting with Mongo DB.
     */
    class MongoException : public std::runtime_error {
    public:
        MongoException(const char *msg, const bson_error_t &err)
                : contextMessage_(msg),
                  bsonMessage_(err.message),
                  std::runtime_error(fmt::format("mng_error({},{})", msg, err.message)) {}
        const std::string bsonMessage_;
        const std::string contextMessage_;
    };
}

#endif //KNOWROB_MONGO_EXCEPTION_H
