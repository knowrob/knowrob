//
// Created by daniel on 22.04.23.
//

#ifndef KNOWROB_QUERY_ENGINE_H
#define KNOWROB_QUERY_ENGINE_H

#include <memory>
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/semweb/GraphQuery.h"

namespace knowrob {
    enum QueryFlag {
        QUERY_FLAG_ALL_SOLUTIONS     = 1 << 0,
        QUERY_FLAG_ONE_SOLUTION      = 1 << 1,
        QUERY_FLAG_PERSIST_SOLUTIONS = 1 << 2,
        QUERY_FLAG_UNIQUE_SOLUTIONS  = 1 << 3
    };

	class QueryEngine {
	public:
        /**
         * Evaluate a query represented as a vector of literals.
         * The call is non-blocking and returns a stream of answers.
         * @param literals a vector of literals
         * @param label an optional modalFrame label
         * @return a stream of query results
         */
        virtual AnswerBufferPtr submitQuery(const GraphQueryPtr &graphQuery) = 0;

        /**
         * Evaluate a query represented as a Formula.
         * The call is non-blocking and returns a stream of answers.
         * @param query a formula
         * @return a stream of query results
         */
        virtual AnswerBufferPtr submitQuery(const FormulaPtr &query, int queryFlags) = 0;

        /**
         * Evaluate a query represented as a Literal.
         * The call is non-blocking and returns a stream of answers.
         * @param query a literal
         * @return a stream of query results
         */
        virtual AnswerBufferPtr submitQuery(const LiteralPtr &query, int queryFlags) = 0;

        /**
         * Evaluate a query represented as a LabeledLiteral.
         * The call is non-blocking and returns a stream of answers.
         * @param query a labeled literal
         * @return a stream of query results
         */
        virtual AnswerBufferPtr submitQuery(const LabeledLiteralPtr &query, int queryFlags) = 0;
	};
}

#endif //KNOWROB_QUERY_ENGINE_H
