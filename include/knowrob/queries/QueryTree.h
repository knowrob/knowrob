//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_QUERY_TREE_H
#define KNOWROB_QUERY_TREE_H

#include <memory>
#include <list>
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/modalities/ModalityLabel.h"

namespace knowrob {
    /**
     * Constructs a tableau-like tree by decomposing an input formula.
     */
    class QueryTree {
    public:
        explicit QueryTree(const FormulaPtr &query);

        ~QueryTree();

        QueryTree(const QueryTree&) = delete;

        /**
         * @return number of paths from root of the tree to leafs.
         */
        auto numPaths() const { return paths_.size(); }

        /**
         * @return list of paths from root of the tree to leafs.
         */
        const auto& paths() const { return paths_; }

        /**
         * @return begin iterator over paths from root to leafs.
         */
        auto begin() const { return paths_.begin(); }

        /**
         * @return end iterator over paths from root to leafs.
         */
        auto end() const { return paths_.end(); }


        /**
         * A path in a QueryTree from root of the tree to a leaf.
         */
        class Path {
        public:
            Path() = default;

            /**
             * @return number of literals in this path
             */
            auto numLiterals() const { return literals_.size(); }

            /**
             * @return list of literals
             */
            const auto& literals() const { return literals_; }

            /**
             * @return begin iterator over literals in a path.
             */
            auto begin() const { return literals_.begin(); }

            /**
             * @return end iterator over literals in a path.
             */
            auto end() const { return literals_.end(); }

        protected:
            std::list<LabeledLiteral> literals_;
            friend class QueryTree;
        };

    protected:
        const FormulaPtr query_;
        std::list<Path> paths_;

        class Node {
        public:
            Node(Node *parent,
                 const ModalityLabelPtr &label,
                 const FormulaPtr &formula,
                 bool isNegated);
            Node *parent;
            const ModalityLabelPtr label;
            const FormulaPtr formula;
            bool isNegated;
            bool isOpen;
            std::list<Node*> successors;

            int priority() const;
        };

        Node* rootNode_;

        struct NodeComparator {
            bool operator()(const Node *a, const Node *b) const;
        };
        std::priority_queue<Node*, std::vector<Node*>, NodeComparator> openNodes_;

        Node* createNode(Node *parent,
                         const ModalityLabelPtr &modalities,
                         const FormulaPtr &phi,
                         bool isNegated);

        static std::list<QueryTree::Node*> getLeafs(Node *n);

        static bool hasCompletePath(Node *leaf);

        static void constructPath(Node *leaf, Path &path);

        void expandNextNode();
    };

} // knowrob

#endif //KNOWROB_QUERY_TREE_H
