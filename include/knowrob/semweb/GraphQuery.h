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
#include "knowrob/semweb/TripleExpression.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/modalities/ModalFrame.h"

namespace knowrob {

    class GraphQuery {
    public:
        /**
         * A path query constructed from a sequence of literals.
         * All literals in the sequence are considered to be in a conjunction with each other.
         * @param literals an ordered sequence of literals.
         * @param label an optional label of the literals
         */
        GraphQuery(const std::vector<LiteralPtr> &literals, int flags, ModalFrame modalFrame=ModalFrame())
        : literals_(literals), modalFrame_(std::move(modalFrame)), flags_(flags) {}

        /**
         * A literal query. Answers are instantiations of the literal where free variables
         * have been replaced by values.
         * @param literals an ordered sequence of literals.
         * @param label an optional label of the literals
         */
        GraphQuery(LiteralPtr &literal, int flags, ModalFrame modalFrame=ModalFrame())
        : literals_({literal}), modalFrame_(std::move(modalFrame)), flags_(flags) {}

        const auto& literals() const { return literals_; }

        const auto& modalFrame() const { return modalFrame_; }

        const auto& flags() const { return flags_; }

        std::list<semweb::TripleExpression> asTripleExpression();

    protected:
        const std::vector<LiteralPtr> literals_;
        const ModalFrame modalFrame_;
        const int flags_;
    };

    using GraphQueryPtr = std::shared_ptr<GraphQuery>;

} // knowrob

#endif //KNOWROB_GRAPH_QUERY_H
