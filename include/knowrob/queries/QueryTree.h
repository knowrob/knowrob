//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_QUERY_TREE_H
#define KNOWROB_QUERY_TREE_H

#include <memory>
#include <list>
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/AtomicProposition.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/NegatedFormula.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/ModalLiteral.h"
#include "knowrob/terms/NestedModality.h"

namespace knowrob {
    /**
     * Constructs a tableau-like tree by decomposing an input query.
     * Each path from root to a leaf corresponds to a conjunctive query
     * whose models are also models of the input query.
     * Different paths corresponds to a disjunction, i.e. the tree
     * translates the query into disjunctive normalform.
     */
    class QueryTree {
    public:
        explicit QueryTree(const FormulaPtr &query);

        ~QueryTree();

        QueryTree(const QueryTree&) = delete;

        /**
         * @return begin iterator over paths from root to leafs.
         */
        auto begin() const { return paths_.begin(); }

        /**
         * @return end iterator over paths from root to leafs.
         */
        auto end() const { return paths_.end(); }

    protected:
        const FormulaPtr query_;
        std::list<std::list<ModalLiteral>> paths_;

        class Node {
        public:
            Node(Node *parent,
                 const ModalitySequencePtr &modalities,
                 const FormulaPtr &formula,
                 bool isNegated);
            Node *parent;
            const ModalitySequencePtr modalities;
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
                         const ModalitySequencePtr &modalities,
                         const FormulaPtr &phi,
                         bool isNegated);

        static std::list<QueryTree::Node*> getLeafs(Node *n);

        static bool hasClosedPath(Node *leaf);

        static void constructConjunction(Node *leaf, std::list<ModalLiteral> &path);

        void expandNextNode();
        void expandNode(Node* n, Conjunction *formula);
        void expandNode(Node* n, Disjunction *formula);
        void expandNode(Node* n, Implication *formula);
        void expandNode(Node* n, NegatedFormula *formula);
        void expandNode(Node* n, ModalFormula *formula);
    };

} // knowrob

#endif //KNOWROB_QUERY_TREE_H
