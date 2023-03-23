//
// Created by daniel on 21.03.23.
//

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryTree.h"

using namespace knowrob;

QueryTree::QueryTree(const FormulaPtr &query)
: rootNode_(new Node(nullptr, std::make_shared<NestedModality>(), query, false))
{
    openNodes_.push(rootNode_);
    while(!openNodes_.empty()) {
        expandNextNode();
    }
}

QueryTree::~QueryTree()
{
    std::list<Node*> fifo;
    fifo.push_back(rootNode_);
    while(!fifo.empty()) {
        Node *next = fifo.front();
        fifo.pop_front();
        for (auto *x: next->successors) {
            fifo.push_back(x);
        }
        delete next;
    }
    rootNode_ = nullptr;
}

QueryTree::Node::Node(Node *parent,
                      const ModalitySequencePtr &modalities,
                      const FormulaPtr &formula,
                      bool isNegated)
: parent(parent),
  modalities(modalities),
  formula(formula),
  isNegated(isNegated),
  isOpen(true)
{
}

int QueryTree::Node::priority() const
{
    switch(formula->type()) {
        case FormulaType::MODAL:
        case FormulaType::NEGATION:
        case FormulaType::PREDICATE:
            return 1;
        case FormulaType::CONJUNCTION:
            return isNegated ? 0 : 1;
        case FormulaType::DISJUNCTION:
        case FormulaType::IMPLICATION:
            return isNegated ? 1 : 0;
        default:
            KB_WARN("Ignoring formula {} of unknown type {}.",
                    *formula, (int)formula->type());
            break;
    }
	return 0;
}

bool QueryTree::NodeComparator::operator()(const Node *a, const Node *b) const
{
    int priority_a = a->priority();
    int priority_b = b->priority();
    if(priority_a != priority_b) {
        return priority_a < priority_b;
    }
    else {
        return a < b;
    }
}

std::list<QueryTree::Node*> QueryTree::getLeafs(Node *n)
{
    std::list<Node*> out;
    std::list<Node*> fifo;
    fifo.push_back(n);
    while(!fifo.empty()) {
         Node *next = fifo.front();
         fifo.pop_front();
         if(next->successors.empty()) {
             out.push_back(next);
         }
         else {
             for(auto *x : next->successors) {
                 fifo.push_back(x);
             }
         }
    }
    return out;
}

QueryTree::Node* QueryTree::createNode(Node *parent,
                                       const ModalitySequencePtr &modalities,
                                       const FormulaPtr &phi,
                                       bool isNegated)
{
    Node *newNode = new Node(parent, modalities, phi, isNegated);
    parent->successors.push_back(newNode);
    openNodes_.push(newNode);
    return newNode;
}

bool QueryTree::hasClosedPath(Node *leaf)
{
    Node *parent = leaf;
    do {
        if(parent->isOpen) return false;
        parent = parent->parent;
    } while(parent);
    return true;
}

void QueryTree::constructConjunction(Node *leaf, std::list<ModalLiteral> &path)
{
    Node *parent = leaf;
    do {
        if(parent->formula->type() == FormulaType::PREDICATE) {
            auto phi = std::dynamic_pointer_cast<Predicate>(parent->formula);
            path.emplace_back(parent->modalities, phi, parent->isNegated);
        }
        parent = parent->parent;
    } while(parent);
}

void QueryTree::expandNextNode()
{
    auto next = openNodes_.top();
    next->isOpen = false;
    openNodes_.pop();

    switch(next->formula->type()) {
        case FormulaType::PREDICATE:
            for(Node *leaf : getLeafs(next)) {
                if(hasClosedPath(leaf)) {
                    paths_.emplace_back();
                    auto &path = paths_.back();
                    constructConjunction(leaf, path);
                }
            }
            break;
        case FormulaType::CONJUNCTION:
            expandNode(next, (Conjunction*)next->formula.get());
            break;
        case FormulaType::DISJUNCTION:
            expandNode(next, (Disjunction*)next->formula.get());
            break;
        case FormulaType::IMPLICATION:
            expandNode(next, (Implication*)next->formula.get());
            break;
        case FormulaType::NEGATION:
            expandNode(next, (NegatedFormula*)next->formula.get());
            break;
        case FormulaType::MODAL:
            expandNode(next, (ModalFormula*)next->formula.get());
            break;
        default:
            KB_WARN("Ignoring formula {} of unknown type {}.",
                    *next->formula, (int)next->formula->type());
            break;
    }
}

void QueryTree::expandNode(Node *n, Conjunction *formula)
{
    if(n->isNegated) {
        for(Node *leaf : getLeafs(n)) {
            for(auto &phi : formula->formulae()) {
                createNode(leaf, n->modalities, phi, true);
            }
        }
    }
    else {
        for(Node *leaf : getLeafs(n)) {
            Node *parent = leaf;
            for (auto &phi: formula->formulae()) {
                parent = createNode(parent, n->modalities, phi, false);
            }
        }
    }
}

void QueryTree::expandNode(Node *n, Disjunction *formula)
{
    if(n->isNegated) {
        for(Node *leaf : getLeafs(n)) {
            Node *parent = leaf;
            for (auto &phi: formula->formulae()) {
                parent = createNode(parent, n->modalities, phi, false);
            }
        }
    }
    else {
        for(Node *leaf : getLeafs(n)) {
            for (auto &phi: formula->formulae()) {
                createNode(leaf, n->modalities, phi, true);
            }
        }
    }
}

void QueryTree::expandNode(Node *n, Implication *formula)
{
    if(n->isNegated) {
        for(Node *leaf : getLeafs(n)) {
            Node *parent = leaf;
            parent = createNode(parent, n->modalities, formula->antecedent(), false);
            createNode(parent, n->modalities, formula->consequent(), true);
        }
    }
    else {
        for(Node *leaf : getLeafs(n)) {
            createNode(leaf, n->modalities, formula->antecedent(), true);
            createNode(leaf, n->modalities, formula->consequent(), false);
        }
    }
}

void QueryTree::expandNode(Node *n, NegatedFormula *formula)
{
    for(Node *leaf : getLeafs(n)) {
        createNode(leaf, n->modalities, formula->negatedFormula(), !n->isNegated);
    }
}

void QueryTree::expandNode(Node *n, ModalFormula *formula)
{
    for(Node *leaf : getLeafs(n)) {
        auto newModalities = std::make_shared<NestedModality>(*n->modalities);
        newModalities->push_back(formula->modalOperator());
        createNode(leaf, newModalities, formula->modalFormula(), !n->isNegated);
    }
}
