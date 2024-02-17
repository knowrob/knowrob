//
// Created by daniel on 08.04.23.
//

#ifndef KNOWROB_MONGO_AGGREGATION_TERMS_H
#define KNOWROB_MONGO_AGGREGATION_TERMS_H

#include <mongoc.h>
#include "knowrob/terms/Term.h"

namespace knowrob::mongo::aggregation
{
    void appendTermQuery(
            bson_t *doc,
            const char *key,
            const TermPtr &term,
            const char *queryOperator=nullptr,
            bool matchNullValue=false);

    void appendArrayQuery(
            bson_t *doc,
            const char *key,
            const std::vector<TermPtr> &terms,
            const char *arrayOperator="$or");
}

#endif //KNOWROB_MONGO_AGGREGATION_TERMS_H
