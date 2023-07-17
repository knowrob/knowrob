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
     * A node in a dependency graph.
     */
    class DependencyNode {
    public:
        DependencyNode() = default;

        /**
         * @return the set of variables appearing in literal nodes.
         */
        virtual const VariableSet& variables() const = 0;

        /**
         * @return number of free variables in this node.
         */
        auto numVariables() const { return variables().size(); }

        /**
         * @return number of modal nodes with shared free variables.
         */
        auto numNeighbors() const { return neighbors_.size(); }

        /**
         * @return nodes with free variables shared with this node.
         */
        const auto& neighbors() const { return neighbors_; }

        void addDependency(const std::shared_ptr<DependencyNode> &other);

    protected:
        std::list<std::shared_ptr<DependencyNode>> neighbors_;
        friend class DependencyGraph;
    };
    using DependencyNodePtr = std::shared_ptr<DependencyNode>;

    /**
     * A node labeled with a literal.
     */
    class LiteralDependencyNode : public DependencyNode {
    public:
        explicit LiteralDependencyNode(const LiteralPtr &literal);

        /**
         * @return the literal associated to this node.
         */
        const auto& literal() const { return literal_; }

        // Override DependencyNode
        const VariableSet& variables() const override { return literal_->predicate()->getVariables(); }

    protected:
        const LiteralPtr literal_;
    };

    /**
     * A node labeled with a modality that contains dependency groups of literals.
     */
    class ModalDependencyNode : public DependencyNode {
    public:
        explicit ModalDependencyNode(const std::list<LiteralPtr> &literals,
                                     const ModalityLabelPtr &label={});

        /**
         * @return the modality label of this node.
         */
        const auto& label() const { return label_; }

        /**
         * @return the literal nodes within this modal node.
         */
        const auto& literals() const { return literals_; }

        /**
         * @return number of nodes with literal label.
         */
        auto numLiterals() const { return literals_.size(); }

        // Override DependencyNode
        const VariableSet& variables() const override { return variables_; }

    protected:
        const std::list<LiteralPtr> literals_;
        const ModalityLabelPtr label_;
        VariableSet variables_;
    };

    /**
     * A group of nodes with dependencies.
     * @tparam NodeType the node type.
     */
    struct DependencyGroup {
        VariableSet variables_;
        std::list<DependencyNodePtr> member_;

        void operator+=(const DependencyGroup &other) {
            member_.insert(other.member_.end(), other.member_.begin(), other.member_.end());
            variables_.insert(other.variables_.begin(), other.variables_.end());
        }
    };

    /**
     * A graph capturing a dependency relation between literals in a formula.
     * Two literals are viewed as dependant in case they share a free variable.
     * Here, labeled literal are considered.
     * The graph is made of nodes labeled with a modality that contain groups of literals.
     * Literals that are part of the same node are all evaluated wrt. the modality label of this node.
     * The dependency relation is rather computed between these modality groups.
     */
    class DependencyGraph {
    public:
        DependencyGraph() = default;

        /**
         * Same as insert(node).
         * @param node a dependency node.
         */
        void operator+=(const DependencyNodePtr &node);

        /**
         * Add a new node to the graph and compute dependency relation
         * with other nodes.
         * @param node a dependency node.
         */
        void insert(const DependencyNodePtr &node);

        /**
         * Add a new node to the graph and compute dependency relation
         * with other nodes.
         * @param literals set of literals considered in conjunction.
         * @param label a modality label.
         */
        void insert(const std::list<LiteralPtr> &literals, const ModalityLabelPtr &label);

        /**
         * Add a new node to the graph and compute dependency relation
         * with other nodes.
         * @param literal a literal.
         */
        void insert(const LiteralPtr &literal);

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
        auto numNodes() const { return nodes_.size(); }

        /**
         * @return number of dependency groups among modality labeled nodes.
         */
        auto numGroups() const { return groups_.size(); }

    protected:
        std::list<DependencyNodePtr> nodes_;
        std::list<DependencyGroup> groups_;
	};

} // knowrob

#endif //KNOWROB_DEPENDENCY_GRAPH_H
