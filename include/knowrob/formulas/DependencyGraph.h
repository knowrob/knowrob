/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEPENDENCY_GRAPH_H
#define KNOWROB_DEPENDENCY_GRAPH_H

#include <list>
#include "knowrob/formulas/FirstOrderLiteral.h"

namespace knowrob {
	/**
	 * A node in a dependency graph labeled with a literal.
	 */
	class DependencyNode {
	public:
		explicit DependencyNode(const FirstOrderLiteralPtr &literal);

		/**
		 * @return the set of variables appearing in literal nodes.
		 */
		const std::set<std::string_view> &variables() const { return literal_->predicate()->variables(); }

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
		const auto &neighbors() const { return neighbors_; }

		void addDependency(const std::shared_ptr<DependencyNode> &other);

		/**
		 * @return the literal associated to this node.
		 */
		const auto &literal() const { return literal_; }

	protected:
		std::list<std::shared_ptr<DependencyNode>> neighbors_;
		const FirstOrderLiteralPtr literal_;

		friend class DependencyGraph;
	};

	using DependencyNodePtr = std::shared_ptr<DependencyNode>;

	/**
	 * A group of nodes with dependencies.
	 * @tparam NodeType the node type.
	 */
	struct DependencyGroup {
		std::set<std::string_view> variables_;
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
	 * The graph is made of nodes labeled with a modalFrame that contain groups of literals.
	 * Literals that are part of the same node are all evaluated wrt. the modalFrame label of this node.
	 * The dependency relation is rather computed between these modalFrame groups.
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
		 * @param literal a literal.
		 */
		void insert(const FirstOrderLiteralPtr &literal);

		/**
		 * Add a new node to the graph and compute dependency relation
		 * with other nodes.
		 * @param literals set of literals considered in conjunction.
		 */
		void insert(const std::vector<FirstOrderLiteralPtr> &literals);

		/**
		 * Insert a node for each iteration.
		 * @tparam Iterator an iterator with ++ and * operator
		 * @param begin marks begin of iteration
		 * @param end marks end of iteration
		 */
		template<typename Iterator>
		void insert(Iterator begin, Iterator end) {
			while (begin != end) {
				auto next = *begin;
				insert(next);
				++begin;
			}
		}

		/**
		 * @return begin iterator over literals in a path.
		 */
		auto begin() const { return groups_.begin(); }

		/**
		 * @return end iterator over literals in a path.
		 */
		auto end() const { return groups_.end(); }

		/**
		 * @return number of nodes with modalFrame label.
		 */
		auto numNodes() const { return nodes_.size(); }

		/**
		 * @return number of dependency groups among modalFrame labeled nodes.
		 */
		auto numGroups() const { return groups_.size(); }

	protected:
		std::list<DependencyNodePtr> nodes_;
		std::list<DependencyGroup> groups_;
	};

} // knowrob

#endif //KNOWROB_DEPENDENCY_GRAPH_H
