//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_DOCUMENT_H
#define KNOWROB_MONGO_DOCUMENT_H

#include <mongoc.h>

namespace knowrob {
    /**
     * A scoped bson document. The memory of the bson_t* is managed by this
     * object, and is freed when the object is destroyed.
     */
    class MongoDocument {
    public:
        explicit MongoDocument(bson_t *bson) : bson_(bson) {}

        MongoDocument(const MongoDocument&) = delete;

        ~MongoDocument() { bson_destroy(bson_); }

        bson_t* bson() const { return bson_; }
    protected:
        bson_t *bson_;
    };

} // knowrob

#endif //KNOWROB_MONGODOCUMENT_H
