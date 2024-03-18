/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <memory>
// KnowRob
#include "knowrob/Logger.h"
#include "knowrob/reasoner/BuiltinEvaluator.h"
#include "knowrob/reasoner/ReasoningGraph.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Conjunction.h"

using namespace knowrob;

template<class T>
std::shared_ptr<T> createConnectiveFormula(
		const FormulaPtr &phi1, const FormulaPtr &phi2, FormulaType type) {
	// attempt to merge if phi1 or phi2 is already a connective formula of the given type
	if (phi1->type() == type) {
		auto *c1 = (T *) phi1.get();
		if (phi2->type() == type) {
			auto *c2 = (T *) phi2.get();
			std::vector<FormulaPtr> args(c1->formulae().size() + c2->formulae().size());
			for (int i = 0; i < c1->formulae().size(); i++) args[i] = c1->formulae()[i];
			for (int i = 0; i < c2->formulae().size(); i++) args[i + c1->formulae().size()] = c2->formulae()[i];
			return std::make_shared<T>(args);
		} else {
			std::vector<FormulaPtr> args(c1->formulae().size() + 1);
			for (int i = 0; i < c1->formulae().size(); i++) args[i] = c1->formulae()[i];
			args[c1->formulae().size()] = phi2;
			return std::make_shared<T>(args);
		}
	} else if (phi2->type() == type) {
		auto *c2 = (T *) phi2.get();
		std::vector<FormulaPtr> args(c2->formulae().size() + 1);
		args[0] = phi1;
		for (int i = 0; i < c2->formulae().size(); i++) args[i + 1] = c2->formulae()[i];
		return std::make_shared<T>(args);
	} else {
		return std::make_shared<T>(T(std::vector<FormulaPtr>({phi1, phi2})));
	}
}


ReasoningGraph::Node::Node(
		const std::shared_ptr<Formula> &phi,
		const std::shared_ptr<DefinedReasoner> &reasoner,
		PredicateType predicateType)
		: phi_(phi),
		  predicateType_(predicateType),
		  reasonerAlternatives_({reasoner}) {
}

ReasoningGraph::Node::Node(
		const std::shared_ptr<Formula> &phi,
		const std::set<std::shared_ptr<DefinedReasoner>> &reasonerChoices,
		PredicateType predicateType)
		: phi_(phi),
		  predicateType_(predicateType),
		  reasonerAlternatives_(reasonerChoices) {
}

/*
bool ReasoningGraph::Node::canBeCombinedWith(
		const NodePtr &other, ReasonerCapability requiredCapability)
{
	for(auto &r1 : reasonerAlternatives_) {
		// reasoner must support the capability
		// and other node must have the same reasoner choice
		if(r1->reasoner()->hasCapability(requiredCapability) &&
		   other->reasonerAlternatives_.find(r1) != other->reasonerAlternatives_.end())
			return true;
	}
	return false;
}
 */

bool ReasoningGraph::Node::isBuiltinSupported(const Predicate &phi) const {
	for (auto &r1: reasonerAlternatives_) {
		if (r1->reasoner()->getPredicateDescription(phi.indicator()) != nullptr)
			return true;
	}
	return false;
}


void ReasoningGraph::addInitialNode(const NodePtr &node) {
	initialNodes_.push_back(node);
	if (node->successors_.empty()) {
		terminalNodes_.push_back(node);
	}
}

void ReasoningGraph::removeNode(const NodePtr &node) {
	if (node->predecessors_.empty()) {
		initialNodes_.remove(node);
	} else {
		for (auto &predecessor: node->predecessors_) {
			predecessor->successors_.remove(node);
			if (node->successors_.empty()) terminalNodes_.push_back(predecessor);
		}
		node->predecessors_.clear();
	}

	if (node->successors_.empty()) {
		terminalNodes_.remove(node);
	} else {
		for (auto &successor: node->successors_) {
			successor->predecessors_.remove(node);
			if (node->predecessors_.empty()) initialNodes_.push_back(successor);
		}
		node->successors_.clear();
	}
}

void ReasoningGraph::addSuccessor(const NodePtr &predecessor, const NodePtr &successor) {
	if (predecessor->successors_.empty()) terminalNodes_.remove(predecessor);
	if (successor->predecessors_.empty()) initialNodes_.remove(successor);

	predecessor->successors_.push_back(successor);
	successor->predecessors_.push_back(predecessor);
}

void ReasoningGraph::disjunction(const ReasoningGraph &other) {
	// gather initial nodes of this graph without successors
	std::list<NodePtr> simpleNodes;
	for (auto &node1: initialNodes_) {
		if (node1->successors_.empty()) simpleNodes.push_back(node1);
	}
	// iterate over initial nodes of the other graph.
	// if the node has no successors it may be combinable into a disjunctive query
	// with a node in simpleNodes.
	for (auto &node2: other.initialNodes()) {
		if (node2->successors_.empty()) {
			bool hasNode = false;
			for (auto &simple1: simpleNodes) {
				if (simple1->canBeCombinedWith(node2, CAPABILITY_DISJUNCTIVE_QUERIES)) {
					std::set<std::shared_ptr<DefinedReasoner>> reasonerChoices;
					for (auto &reasonerChoice: node2->reasonerAlternatives_)
						if (simple1->reasonerAlternatives_.find(reasonerChoice) != simple1->reasonerAlternatives_.end())
							reasonerChoices.insert(reasonerChoice);

					auto phi = createConnectiveFormula<Disjunction>(
							simple1->phi_, node2->phi_, FormulaType::DISJUNCTION);
					auto disjunctiveNode = std::make_shared<Node>(
							phi, reasonerChoices, PredicateType::FORMULA);
					initialNodes_.remove(simple1);
					initialNodes_.push_back(disjunctiveNode);
					simpleNodes.remove(simple1);
					simpleNodes.push_back(disjunctiveNode);
					hasNode = true;
					break;
				}
			}
			if (hasNode) continue;
		}
		initialNodes_.push_back(node2);
	}
}

void ReasoningGraph::conjunction(const ReasoningGraph &other) {
	// create a copy of terminal nodes list as it is modified below
	auto terminals = terminalNodes();
	// look at each pair of terminal node of this, and initial node of the other graph.
	// if possible, create a conjunctive query node, else add edges between the
	// nodes to connect both graphs.
	for (auto &terminal1: terminals) {
		for (auto &initial2: other.initialNodes()) {
			if (terminal1->canBeCombinedWith(initial2, CAPABILITY_CONJUNCTIVE_QUERIES)) {
				// create a conjunctive node (terminal1 AND initial2)
				addConjunctiveNode(terminal1, initial2);
			} else if (isEdgeNeeded(terminal1, initial2)) {
				// terminal1 and initial2 cannot be combined via conjunction.
				// but it could be that one of the nodes refers to a builtin
				// in which case the edge is omittable for some cases.
				addSuccessor(terminal1, initial2);
			}
		}
		// remove dangling node in this graph, i.e. terminal nodes not connected
		// to the other graph.
		// NOTE: dangling nodes in the other graph are not referred to in this graph, so can be ignored
		if (terminal1->successors_.empty()) removeNode(terminal1);
	}
}

void ReasoningGraph::addConjunctiveNode(const NodePtr &a, const NodePtr &b) {
	std::set<std::shared_ptr<DefinedReasoner>> reasonerChoices;
	for (auto &r1: a->reasonerAlternatives_)
		if (b->reasonerAlternatives_.find(r1) != b->reasonerAlternatives_.end()) reasonerChoices.insert(r1);
	// create a conjunctive node
	auto phi = createConnectiveFormula<Conjunction>(
			a->phi_, b->phi_, FormulaType::CONJUNCTION);
	auto conjunctiveNode = std::make_shared<Node>(
			phi, reasonerChoices, PredicateType::FORMULA);

	// connect the node to the rest of the graph
	if (a->predecessors_.empty()) {
		initialNodes_.push_back(conjunctiveNode);
	} else {
		for (auto &x: a->predecessors_)
			addSuccessor(x, conjunctiveNode);
	}
	if (b->successors_.empty()) {
		terminalNodes_.push_back(conjunctiveNode);
	} else {
		for (auto &x: b->successors_)
			addSuccessor(conjunctiveNode, x);
	}
}

bool ReasoningGraph::isEdgeNeeded(const NodePtr &a, const NodePtr &b) {
	if (a->predicateType_ == PredicateType::BUILT_IN) {
		// no edge needed if phi_a is a builtin supported by reasoner_b
		if (b->isBuiltinSupported(*(Predicate *) a->phi_.get()))
			return false;
	}
	if (b->predicateType_ == PredicateType::BUILT_IN) {
		// no edge needed if phi_b is a builtin supported by reasoner_a
		if (a->isBuiltinSupported(*(Predicate *) b->phi_.get()))
			return false;
	}
	return true;
}
