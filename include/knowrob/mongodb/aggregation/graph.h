//
// Created by daniel on 05.04.23.
//

#ifndef KNOWROB_MONGO_GRAPH_AGGREGATION_H
#define KNOWROB_MONGO_GRAPH_AGGREGATION_H

#include <mongoc.h>
#include <string>
#include "knowrob/mongodb/Document.h"
#include "knowrob/mongodb/Pipeline.h"

namespace knowrob::mongo::aggregation
{
    void lookupParents(
            aggregation::Pipeline &pipeline,
            const std::string_view &collectionName,
            const std::string_view &entity,
            const std::string_view &hierarchyRelation);

    void updateHierarchyP(
            aggregation::Pipeline &pipeline,
            const std::string_view &collection,
            const std::string_view &relation,
            const std::string_view &newChild);

    void updateHierarchyO(
            aggregation::Pipeline &pipeline,
            const std::string_view &collectionName,
            const std::string_view &relation,
            const std::string_view &newChild,
            const std::string_view &newParent);

} // knowrob::mongo

#endif //KNOWROB_MONGO_GRAPH_AGGREGATION_H
