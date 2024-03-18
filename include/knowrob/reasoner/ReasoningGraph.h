/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONING_GRAPH_H_
#define KNOWROB_REASONING_GRAPH_H_

#include <list>
#include <memory>
#include "knowrob/formulas/Predicate.h"
#include "knowrob/reasoner/ReasonerManager.h"

namespace knowrob {
	/**
	 * A directed acyclic graph where nodes correspond to steps
	 * in a reasoning pipeline.
	 * The meaning of an edge in the graph is that the predecessor
	 * is evaluated before the successor node, and that instantiations
	 * done during the evaluation of the predecessor node are provided
	 * as a parameter for the evaluation of the successor node.
	 */
	class ReasoningGraph {
	public:
		// forward declaration
		class Node;
		// define aliases
		using NodePtr = std::shared_ptr<Node>;
		using NodeList = std::list<NodePtr>;

		ReasoningGraph() = default;

		/**
		 * Adds a node to this graph list of initial nodes.
		 * @param node a node.
		 */
		void addInitialNode(const NodePtr &node);

		/**
		 * Removes a node from this graph.
		 * @param node a node.
		 */
		void removeNode(const NodePtr &node);

		/**
		 * Computes the conjunction with another graph.
		 * @param other another graph.
		 */
		void conjunction(const ReasoningGraph &other);

		/**
		 * Computes the disjunction with another graph.
		 * @param other another graph.
		 */
		void disjunction(const ReasoningGraph &other);

		/**
		 * Add a directed edge between two nodes.
		 * @param predecessor the predecessor node.
		 * @param successor the successor node.
		 */
		void addSuccessor(const NodePtr &predecessor, const NodePtr &successor);

		/**
		 * @return set of initial nodes.
		 */
		const NodeList &initialNodes() const { return initialNodes_; }

		/**
		 * @return set of terminal nodes.
		 */
		const NodeList &terminalNodes() const { return terminalNodes_; }

		/**
		 * A node in an evaluation graph.
		 */
		class Node {
		public:
			/**
			 * @param phi a formula to be evaluated
			 * @param reasoner a reasoner that can submitQuery the formula
			 * @param predicateType a type identifier.
			 */
			Node(const std::shared_ptr<Formula> &phi,
				 const std::shared_ptr<DefinedReasoner> &reasoner,
				 PredicateType predicateType);

			/**
			 * @param phi a formula to be evaluated
			 * @param reasoner a list of reasoner that can submitQuery the formula
			 * @param predicateType a type identifier.
			 */
			Node(const std::shared_ptr<Formula> &phi,
				 const std::set<std::shared_ptr<DefinedReasoner>> &reasonerChoices,
				 PredicateType predicateType);

			/**
			 * @param phi a builtin predicate
			 * @return true if reasoner of this node define given builtin
			 */
			bool isBuiltinSupported(const Predicate &phi) const;

			/**
			 * @param other another node
			 * @param requiredCapability a capability flag
			 * @return true if this node can be combined with another via a reasoner capability
			 */
			//bool canBeCombinedWith(const NodePtr &other, ReasonerCapability requiredCapability);

			/**
			 * @return the formula associated to this node.
			 */
			const std::shared_ptr<Formula> &phi() const { return phi_; }

			/**
			 * @return the predicate type associated to this node.
			 */
			PredicateType predicateType() const { return predicateType_; }

			/**
			 * @return the reasoner associated to this node.
			 */
			const std::set<std::shared_ptr<DefinedReasoner>> &reasonerAlternatives() { return reasonerAlternatives_; }

			/**
			 * @return list of node successors.
			 */
			const NodeList &successors() const { return successors_; }

			/**
			 * @return list of node predecessors.
			 */
			const NodeList &predecessors() const { return predecessors_; }

		protected:
			std::shared_ptr<Formula> phi_;
			PredicateType predicateType_;
			std::set<std::shared_ptr<DefinedReasoner>> reasonerAlternatives_;
			std::list<std::shared_ptr<Node>> successors_;
			std::list<std::shared_ptr<Node>> predecessors_;

			friend class ReasoningGraph;
		};

	protected:
		NodeList initialNodes_;
		NodeList terminalNodes_;

		static bool isEdgeNeeded(const NodePtr &a, const NodePtr &b);

		void addConjunctiveNode(const NodePtr &a, const NodePtr &b);
	};
}

#endif //KNOWROB_REASONING_GRAPH_H_
