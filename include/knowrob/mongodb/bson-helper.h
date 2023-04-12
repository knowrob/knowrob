//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_BSON_H
#define KNOWROB_BSON_H

#include <mongoc.h>

namespace knowrob::mongo {
    /**
     * A wrapper for bson_t used to avoid warnings reported by
     * gcc about using bson_t as a template argument.
     */
    typedef struct {
        bson_t bson;
    } bson_wrapper;

    /**
     * A wrapper for bson_t used to avoid warnings reported by
     * gcc about using bson_t as a template argument.
     */
    typedef struct {
        const bson_t *bson;
    } bson_wrapper_ptr;
}

#endif //KNOWROB_BSON_H
