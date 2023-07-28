//
// Created by daniel on 08.04.23.
//

#ifndef KNOWROB_MONGO_AGGREGATION_TRIPLES_H
#define KNOWROB_MONGO_AGGREGATION_TRIPLES_H

#include <mongoc.h>
#include <string>
#include <optional>
#include "knowrob/terms/Term.h"
#include "knowrob/mongodb/Pipeline.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/formulas/FramedLiteral.h"

namespace knowrob::mongo::aggregation
{
    struct TripleLookupData {
        explicit TripleLookupData(const semweb::FramedLiteral *expr)
        : expr(expr),
          maxNumOfTriples(0),
          mayHasMoreGroundings(true),
          forceTransitiveLookup(false)
          {}
        const semweb::FramedLiteral *expr;
        uint32_t maxNumOfTriples;
        std::set<std::string_view> knownGroundedVariables;
        bool mayHasMoreGroundings;
        bool forceTransitiveLookup;
    };

    void appendTripleSelector(
                bson_t *selectorDoc,
                const semweb::FramedLiteral &tripleExpression,
                bool b_isTaxonomicProperty);

    void appendGraphSelector(
                bson_t *selectorDoc,
                const semweb::FramedLiteral &tripleExpression);

    void appendAgentSelector(
                bson_t *selectorDoc,
                const semweb::FramedLiteral &tripleExpression);

    void appendTimeSelector(
                bson_t *selectorDoc,
                const semweb::FramedLiteral &tripleExpression);

    void appendConfidenceSelector(
                bson_t *selectorDoc,
                const semweb::FramedLiteral &tripleExpression);

    void lookupTriple(
            aggregation::Pipeline &pipeline,
            const std::string_view &collection,
            const std::shared_ptr<semweb::Vocabulary> &vocabulary,
            const TripleLookupData &lookupData);

    void lookupTriplePaths(
            aggregation::Pipeline &pipeline,
            const std::string_view &collection,
            const std::shared_ptr<semweb::Vocabulary> &vocabulary,
            const std::list<semweb::FramedLiteral> &tripleExpressions);
}

#endif //KNOWROB_MONGO_AGGREGATION_TRIPLES_H
