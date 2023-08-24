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
#include "knowrob/semweb/FramedRDFLiteral.h"

namespace knowrob::mongo::aggregation
{
    struct TripleLookupData {
        explicit TripleLookupData(const FramedRDFLiteral *expr)
        : expr(expr),
          maxNumOfTriples(0),
          mayHasMoreGroundings(true),
          forceTransitiveLookup(false)
          {}
        const FramedRDFLiteral *expr;
        uint32_t maxNumOfTriples;
        std::set<std::string_view> knownGroundedVariables;
        bool mayHasMoreGroundings;
        bool forceTransitiveLookup;
    };

    void appendTripleSelector(
                bson_t *selectorDoc,
                const FramedRDFLiteral &tripleExpression,
                bool b_isTaxonomicProperty);

    void appendGraphSelector(
                bson_t *selectorDoc,
                const FramedRDFLiteral &tripleExpression);

    void appendEpistemicSelector(
                bson_t *selectorDoc,
                const FramedRDFLiteral &tripleExpression);

    void appendTimeSelector(
                bson_t *selectorDoc,
                const FramedRDFLiteral &tripleExpression);

    void lookupTriple(
            aggregation::Pipeline &pipeline,
            const std::string_view &collection,
            const std::shared_ptr<semweb::Vocabulary> &vocabulary,
            const TripleLookupData &lookupData);

    void lookupTriplePaths(
            aggregation::Pipeline &pipeline,
            const std::string_view &collection,
            const std::shared_ptr<semweb::Vocabulary> &vocabulary,
            const std::vector<FramedRDFLiteralPtr> &tripleExpressions);
}

#endif //KNOWROB_MONGO_AGGREGATION_TRIPLES_H
