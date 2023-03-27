/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEPENDENCY_GRAPH_H
#define KNOWROB_DEPENDENCY_GRAPH_H

#include "knowrob/modalities/ModalityLabel.h"
#include "knowrob/formulas/Literal.h"

namespace knowrob {

    /**
     * A group of nodes with dependencies.
     * @tparam NodeType the node type.
     */
    template <class NodeType> struct DependencyGroup {
        VariableSet variables_;
        std::list<NodeType*> member_;

        void operator+=(const DependencyGroup &other) {
            member_.insert(other.member_.end(), other.member_.begin(), other.member_.end());
            variables_.insert(other.variables_.begin(), other.variables_.end());
        }
    };

    class DependencyNode {
    public:
        DependencyNode() = default;
    };

    /**
     * A node labeled with a literal.
     */
    class LiteralDependencyNode : public DependencyNode {
    public:
        explicit LiteralDependencyNode(const LiteralPtr &literal);

        /**
         * @return the literal associated to this node.
         */
        const auto literal() const { return literal_; }

    protected:
        const LiteralPtr literal_;
    };

    /**
     * A node labeled with a modality that contains a graph
     * of literal nodes.
     */
    class ModalDependencyNode : public DependencyNode {
    public:
        ModalDependencyNode(const ModalityLabelPtr &label, const std::list<LiteralPtr> &literals);

        /**
         * @return the modality label of this node.
         */
        const auto& label() const { return label_; }

        /**
         * @return the set of variables appearing in literal nodes.
         */
        const auto& variables() const { return variables_; }

        /**
         * @return number of nodes with literal label.
         */
        auto numLiteralNodes() const { return nodes_.size(); }

        /**
         * @return number of dependency groups among literal labeled nodes.
         */
        auto numLiteralGroups() const { return groups_.size(); }

        auto numNeighbors() const { return neighbors_.size(); }

        auto numVariables() const { return variables_.size(); }

        const auto& literalNodes() const { return nodes_; }

        const auto& literalGroups() const { return groups_; }

        const auto& neighbors() const { return neighbors_; }

    protected:
        const ModalityLabelPtr label_;
        std::list<LiteralDependencyNode> nodes_;
        std::list<DependencyGroup<LiteralDependencyNode>> groups_;
        std::list<ModalDependencyNode*> neighbors_;
        VariableSet variables_;
        friend class DependencyGraph;
    };

    /**
     * A graph capturing the dependency between literals in a query.
     * Two literals are seen as dependant in case they share a free variable.
     */
	class DependencyGraph {
    public:
        DependencyGraph() = default;

        /**
         * Create one node labeled by a modality consisting of
         * a set of literal nodes.
         * @param label the label for the modal formula node.
         * @param literals the labels of literal nodes.
         */
        void addNodes(const ModalityLabelPtr &label, const std::list<LiteralPtr> &literals);

        /**
         * @return begin iterator over literals in a path.
         */
        auto begin() const { return groups_.begin(); }

        /**
         * @return end iterator over literals in a path.
         */
        auto end() const { return groups_.end(); }

        /**
         * @return number of nodes with modality label.
         */
        auto numModalNodes() const { return nodes_.size(); }

        /**
         * @return number of dependency groups among modality labeled nodes.
         */
        auto numModalGroups() const { return groups_.size(); }

        /**
         * @return number of nodes with literal label.
         */
        unsigned long numLiteralNodes() const;

        /**
         * @return number of dependency groups among literal labeled nodes.
         */
        unsigned long numLiteralGroups() const;

    protected:
        std::list<ModalDependencyNode> nodes_;
        std::list<DependencyGroup<ModalDependencyNode>> groups_;
	};

} // knowrob

#endif //KNOWROB_DEPENDENCY_GRAPH_H
