//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_BSON_H
#define KNOWROB_BSON_H

#include <mongoc.h>

namespace knowrob::mongo {
    typedef struct {
        bson_t bson;
    } bson_wrapper;

    typedef struct {
        const bson_t *bson;
    } bson_wrapper_ptr;
}

#endif //KNOWROB_BSON_H
