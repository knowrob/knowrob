//
// Created by daniel on 04.04.23.
//

#ifndef KNOWROB_GRAPH_QUERY_H
#define KNOWROB_GRAPH_QUERY_H

#include "memory"
#include "vector"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Literal.h"

namespace knowrob {

    class GraphQuery {
    public:
        /**
         * A path query constructed from a sequence of literals.
         * All literals in the sequence are considered to be in a conjunction with each other.
         * @param literals an ordered sequence of literals.
         * @param label an optional label of the literals
         */
        explicit GraphQuery(const std::vector<LiteralPtr> &literals, const FormulaLabelPtr &label={})
        : literals_(literals), label_(label) {}

        /**
         * A literal query. Answers are instantiations of the literal where free variables
         * have been replaced by values.
         * @param literals an ordered sequence of literals.
         * @param label an optional label of the literals
         */
        explicit GraphQuery(LiteralPtr &literal, const FormulaLabelPtr &label={})
        : literals_({literal}), label_(label) {}

        const std::vector<LiteralPtr>& literals() const { return literals_; }

        const FormulaLabelPtr& label() const { return label_; }

    protected:
        const std::vector<LiteralPtr> literals_;
        const FormulaLabelPtr label_;
    };

    using GraphQueryPtr = std::shared_ptr<GraphQuery>;

} // knowrob

#endif //KNOWROB_GRAPH_QUERY_H
