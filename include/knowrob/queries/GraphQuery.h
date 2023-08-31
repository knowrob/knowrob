//
// Created by daniel on 04.04.23.
//

#ifndef KNOWROB_GRAPH_QUERY_H
#define KNOWROB_GRAPH_QUERY_H

#include <utility>

#include "memory"
#include "vector"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/queries/Query.h"
#include "knowrob/formulas/Conjunction.h"

namespace knowrob {
    /**
     * A query associated to a particular graph which is identified
     * by a ModalityFrame.
     */
    class GraphQuery : public Query {
    public:
        /**
         * A path query constructed from a sequence of literals.
         * All literals in the sequence are considered to be in a conjunction with each other.
         * @param literals an ordered sequence of literals.
         * @param label an optional label of the literals
         */
        GraphQuery(const std::vector<RDFLiteralPtr> &literals, int flags);

        /**
         * A literal query. Answers are instantiations of the literal where free variables
         * have been replaced by values.
         * @param literals an ordered sequence of literals.
         * @param label an optional label of the literals
         */
        GraphQuery(const RDFLiteralPtr &literal, int flags);

        const auto& literals() const { return literals_; }

        // Override Query
		const FormulaPtr& formula() const override;

        // Override Query
        std::ostream& print(std::ostream &os) const override;

    protected:
        std::vector<RDFLiteralPtr> literals_;
        FormulaPtr formula_;

        void init();
    };

    using GraphQueryPtr = std::shared_ptr<GraphQuery>;

} // knowrob

#endif //KNOWROB_GRAPH_QUERY_H
