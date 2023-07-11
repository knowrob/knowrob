//
// Created by daniel on 22.04.23.
//

#ifndef KNOWROB_QUERY_ENGINE_H
#define KNOWROB_QUERY_ENGINE_H

#include <memory>
#include "knowrob/queries/BufferedAnswers.h"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Literal.h"

namespace knowrob {
    enum class QueryFlag {
        ALL_SOLUTIONS   = 1 << 0,
        ONE_SOLUTION    = 1 << 1
    };

	class QueryEngine {
	public:
        /**
         * Evaluate a query represented as a vector of literals.
         * The call is non-blocking and returns a stream of answers.
         * @param literals a vector of literals
         * @param label an optional modality label
         * @return a stream of query results
         */
        virtual BufferedAnswersPtr submitQuery(
                    const std::vector<LiteralPtr> &literals,
                    const ModalityLabelPtr &label,
                    int queryFlags) = 0;

        /**
         * Evaluate a query represented as a Formula.
         * The call is non-blocking and returns a stream of answers.
         * @param query a formula
         * @return a stream of query results
         */
        virtual BufferedAnswersPtr submitQuery(const FormulaPtr &query, int queryFlags) = 0;

        /**
         * Evaluate a query represented as a Literal.
         * The call is non-blocking and returns a stream of answers.
         * @param query a literal
         * @return a stream of query results
         */
        virtual BufferedAnswersPtr submitQuery(const LiteralPtr &query, int queryFlags) = 0;

        /**
         * Evaluate a query represented as a LabeledLiteral.
         * The call is non-blocking and returns a stream of answers.
         * @param query a labeled literal
         * @return a stream of query results
         */
        virtual BufferedAnswersPtr submitQuery(const LabeledLiteralPtr &query, int queryFlags) = 0;
	};
}

#endif //KNOWROB_QUERY_ENGINE_H
