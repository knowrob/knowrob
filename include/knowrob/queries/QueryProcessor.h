//
// Created by daniel on 26.03.23.
//

#ifndef KNOWROB_QUERY_PROCESSOR_H
#define KNOWROB_QUERY_PROCESSOR_H

#include "knowrob/formulas/Formula.h"
#include "QueryResult.h"
#include "QueryResultQueue.h"
#include "QueryResultBroadcaster.h"
#include "QueryTree.h"
#include "DependencyGraph.h"
#include "knowrob/KnowledgeBase.h"
#include "QueryPipeline.h"

namespace knowrob {

    class QueryProcessor {
    public:
        explicit QueryProcessor(const KnowledgeBasePtr &kb);

        /**
         * Evaluate a query represented as a Formula.
         * The call is non-blocking and returns a queue under construction.
         * @param query a formula
         * @return a queue of query results
         */
        std::shared_ptr<QueryResultQueue> operator<<(const FormulaPtr &query);

        /**
         * Evaluate a query represented as a Literal.
         * The call is non-blocking and returns a queue under construction.
         * @param query a literal
         * @return a queue of query results
         */
        std::shared_ptr<QueryResultQueue> operator<<(const LiteralPtr &query);

        /**
         * Evaluate a query represented as a LabeledLiteral.
         * The call is non-blocking and returns a queue under construction.
         * @param query a labeled literal
         * @return a queue of query results
         */
        std::shared_ptr<QueryResultQueue> operator<<(const LabeledLiteralPtr &query);

    protected:
        KnowledgeBasePtr kb_;
        std::list<QueryPipeline> pipelines_;
    };

} // knowrob

#endif //KNOWROB_QUERY_PROCESSOR_H
