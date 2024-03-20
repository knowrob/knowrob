//
// Created by daniel on 21.03.23.
//

#include <gtest/gtest.h>

#include <utility>

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/formulas/Top.h"

using namespace knowrob;
using namespace knowrob::modals;

QueryTree::QueryTree(const FormulaPtr &query)
: rootNode_(new Node(nullptr, query, false))
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

QueryTree::Node::Node(Node *parent, FormulaPtr formula, bool isNegated)
: parent(parent),
  formula(std::move(formula)),
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

QueryTree::Node* QueryTree::createNode(Node *parent, const FormulaPtr &phi, bool isNegated)
{
    Node *newNode = new Node(parent, phi, isNegated);
    parent->successors.push_back(newNode);
    openNodes_.push(newNode);
    return newNode;
}

bool QueryTree::hasCompletePath(Node *leaf)
{
    Node *parent = leaf;
    do {
        if(parent->isOpen) return false;
        parent = parent->parent;
    } while(parent);
    return true;
}

void QueryTree::constructPath(Node *leaf, Path &path)
{
    Node *parent = leaf;
    do {
        if(parent->formula->type() == FormulaType::PREDICATE ||
           parent->formula->type() == FormulaType::MODAL) {
            if(parent->isNegated) {
            	path.nodes_.push_back(std::make_shared<Negation>(parent->formula));
            }
            else {
            	path.nodes_.push_back(parent->formula);
            }
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
        case FormulaType::MODAL:
            for(Node *leaf : getLeafs(next)) {
                if(hasCompletePath(leaf)) {
                    paths_.emplace_back();
                    auto &path = paths_.back();
                    constructPath(leaf, path);
                }
            }
            break;

        case FormulaType::CONJUNCTION: {
            auto *formula = (Conjunction*)next->formula.get();
            if(next->isNegated) {
                for(Node *leaf : getLeafs(next)) {
                    for(auto &phi : formula->formulae()) {
                        createNode(leaf, phi, true);
                    }
                }
            }
            else {
                for(Node *leaf : getLeafs(next)) {
                    Node *parent = leaf;
                    for (auto &phi: formula->formulae()) {
                        parent = createNode(parent, phi, false);
                    }
                }
            }
            break;
        }

        case FormulaType::DISJUNCTION: {
            auto *formula = (Disjunction*)next->formula.get();
            if(next->isNegated) {
                for(Node *leaf : getLeafs(next)) {
                    Node *parent = leaf;
                    for (auto &phi: formula->formulae()) {
                        parent = createNode(parent, phi, true);
                    }
                }
            }
            else {
                for(Node *leaf : getLeafs(next)) {
                    for (auto &phi: formula->formulae()) {
                        createNode(leaf, phi, false);
                    }
                }
            }
            break;
        }

        case FormulaType::IMPLICATION: {
            auto *formula = (Implication*)next->formula.get();
            if(next->isNegated) {
                for(Node *leaf : getLeafs(next)) {
                    Node *parent = leaf;
                    parent = createNode(parent, formula->antecedent(), false);
                    createNode(parent, formula->consequent(), true);
                }
            }
            else {
                for(Node *leaf : getLeafs(next)) {
                    createNode(leaf, formula->antecedent(), true);
                    createNode(leaf, formula->consequent(), false);
                }
            }
            break;
        }

        case FormulaType::NEGATION: {
            auto *formula = (Negation*)next->formula.get();
            for(Node *leaf : getLeafs(next)) {
                createNode(leaf, formula->negatedFormula(), !next->isNegated);
            }
            break;
        }
    }
}

std::shared_ptr<Formula> QueryTree::Path::toFormula() const
{
	if(nodes_.empty()) {
		return Top::get();
	}
	else if(nodes_.size()==1) {
		return nodes_.front();
	}
	else {
		return std::make_shared<Conjunction>(nodes_);
	}
}


// fixture class for testing
namespace knowrob::testing {
	class QueryTreeTest : public ::testing::Test {
	protected:
		FormulaPtr p_, q_, r_;
		void SetUp() override {
			p_ = std::make_shared<Predicate>("p");
			q_ = std::make_shared<Predicate>("q");
			r_ = std::make_shared<Predicate>("r");
		}
		// void TearDown() override {}
	};
}
using namespace knowrob::testing;

TEST_F(QueryTreeTest, PositiveLiteral)
{
    QueryTree qt(p_);
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(*path.toFormula(), *p_);
        EXPECT_EQ(path.numNodes(), 1);
    }
}

TEST_F(QueryTreeTest, NegativeLiteral)
{
    QueryTree qt(~p_);
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(*path.toFormula(), *(~p_));
        EXPECT_EQ(path.numNodes(), 1);
    }
}

TEST_F(QueryTreeTest, LiteralWithModality)
{
    QueryTree qt(K(p_));
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(*path.toFormula(), *K(p_));
        EXPECT_EQ(path.numNodes(), 1);
    }
}

TEST_F(QueryTreeTest, NestedModality)
{
    QueryTree qt(K(B(~p_)));
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(*path.toFormula(), *K(B(~p_)));
        EXPECT_EQ(path.numNodes(), 1);
    }
}

TEST_F(QueryTreeTest, Conjunction_pq)
{
    QueryTree qt(p_ & q_);
    EXPECT_EQ(qt.numPaths(), 1);
    if(qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numNodes(), 2);
        EXPECT_EQ(*path.toFormula(), *(p_ & q_));
    }
}

TEST_F(QueryTreeTest, Conjunction_pqr)
{
    QueryTree qt(~p_ & q_ & r_);
    EXPECT_EQ(qt.numPaths(), 1);
    if(qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numNodes(), 3);
        EXPECT_EQ(*path.toFormula(), *(~p_ & q_ & r_));
    }
}

TEST_F(QueryTreeTest, Disjunction_pq)
{
    QueryTree qt(p_ | q_);
    EXPECT_EQ(qt.numPaths(), 2);
    if(qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numNodes(), 1);
    }
}

TEST_F(QueryTreeTest, Disjunction_pqr)
{
    QueryTree qt(~p_ | q_ | r_);
    EXPECT_EQ(qt.numPaths(), 3);
    if(qt.numPaths()==3) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numNodes(), 1);
    }
}

TEST_F(QueryTreeTest, AndOr)
{
    QueryTree qt((p_ | ~r_) & (q_ | r_));
    EXPECT_EQ(qt.numPaths(), 4);
    if(qt.numPaths()==4) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numNodes(), 2);
    }
}

TEST_F(QueryTreeTest, ModalAndOr)
{
    QueryTree qt(P(p_ | ~r_) & P(q_ | r_));
    EXPECT_EQ(qt.numPaths(), 1);
}

TEST_F(QueryTreeTest, OrAnd)
{
    QueryTree qt((p_ & ~r_) | (q_ & r_));
    EXPECT_EQ(qt.numPaths(), 2);
    if(qt.numPaths() > 0) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numNodes(), 2);
    }
}

