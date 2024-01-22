//
// Created by daniel on 21.03.23.
//

#include <gtest/gtest.h>

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/BeliefModality.h"

using namespace knowrob;
using namespace knowrob::modality;

ModalityLabelPtr& emptyLabel() {
    static auto empty =
            std::make_shared<ModalityLabel>(*ModalIteration::emptyIteration());
    return empty;
}

QueryTree::QueryTree(const FormulaPtr &query)
: rootNode_(new Node(nullptr,
        ModalIteration::emptyIteration(),
        query,
        false))
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
                      const std::shared_ptr<ModalIteration> &modalOperators,
                      const FormulaPtr &formula,
                      bool isNegated)
: parent(parent),
  modalOperators(modalOperators),
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
                                       const std::shared_ptr<ModalIteration> &modalOperators,
                                       const FormulaPtr &phi,
                                       bool isNegated)
{
    Node *newNode = new Node(parent, modalOperators, phi, isNegated);
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
        if(parent->formula->type() == FormulaType::PREDICATE) {
            auto phi = std::dynamic_pointer_cast<Predicate>(parent->formula);
            auto label = std::make_shared<ModalityLabel>(*parent->modalOperators);
            path.literals_.push_back(std::make_shared<Literal>(
                    phi, parent->isNegated, label));
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
                        createNode(leaf, next->modalOperators, phi, true);
                    }
                }
            }
            else {
                for(Node *leaf : getLeafs(next)) {
                    Node *parent = leaf;
                    for (auto &phi: formula->formulae()) {
                        parent = createNode(parent, next->modalOperators, phi, false);
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
                        parent = createNode(parent, next->modalOperators, phi, true);
                    }
                }
            }
            else {
                for(Node *leaf : getLeafs(next)) {
                    for (auto &phi: formula->formulae()) {
                        createNode(leaf, next->modalOperators, phi, false);
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
                    parent = createNode(parent, next->modalOperators, formula->antecedent(), false);
                    createNode(parent, next->modalOperators, formula->consequent(), true);
                }
            }
            else {
                for(Node *leaf : getLeafs(next)) {
                    createNode(leaf, next->modalOperators, formula->antecedent(), true);
                    createNode(leaf, next->modalOperators, formula->consequent(), false);
                }
            }
            break;
        }

        case FormulaType::NEGATION: {
            auto *formula = (Negation*)next->formula.get();
            for(Node *leaf : getLeafs(next)) {
                createNode(leaf, next->modalOperators, formula->negatedFormula(), !next->isNegated);
            }
            break;
        }

        case FormulaType::MODAL: {
            auto *formula = (ModalFormula*)next->formula.get();
            for(Node *leaf : getLeafs(next)) {
                // add modality to iteration
                auto modalityIteration =
                        std::make_shared<ModalIteration>(*next->modalOperators);
                *modalityIteration += formula->modalOperator();
                // create a node with updated label
                createNode(leaf,
                           modalityIteration,
                           formula->modalFormula(),
                           next->isNegated);
            }
            break;
        }
    }
}


// fixture class for testing
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


TEST_F(QueryTreeTest, PositiveLiteral)
{
    QueryTree qt(p_);
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() == 1) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 1);
        if (path.numLiterals() == 1) {
            auto &lit = path.literals().front();
            EXPECT_FALSE(lit->isNegated());
            EXPECT_TRUE(*lit->label() == *emptyLabel());
            EXPECT_EQ(lit->functor(), "p");
            EXPECT_EQ(lit->arity(), 0);
        }
    }
}

TEST_F(QueryTreeTest, NegativeLiteral)
{
    QueryTree qt(~p_);
    EXPECT_EQ(qt.numPaths(), 1);
    if(qt.numPaths()==1) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 1);
        if(path.numLiterals() == 1) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(lit->isNegated());
            EXPECT_TRUE(*lit->label() == *emptyLabel());
            EXPECT_EQ(lit->functor(), "p");
            EXPECT_EQ(lit->arity(), 0);
        }
    }
}

TEST_F(QueryTreeTest, LiteralWithModality)
{
    QueryTree qt(K(p_));
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() == 1) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 1);
        if (path.numLiterals() == 1) {
            auto &lit = path.literals().front();
            EXPECT_FALSE(lit->isNegated());
            EXPECT_EQ(lit->functor(), "p");
            EXPECT_EQ(lit->arity(), 0);

            auto label = (ModalityLabel*)(lit->label().get());
            EXPECT_EQ(label->epistemicOperator(), KnowledgeModality::K());
            EXPECT_EQ(label->pastOperator().get(), nullptr);
        }
    }
}

TEST_F(QueryTreeTest, NestedModality)
{
    QueryTree qt(K(B(~p_)));
    EXPECT_EQ(qt.numPaths(), 1);
    if (qt.numPaths() == 1) {
        auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 1);
        if (path.numLiterals() == 1) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(lit->isNegated());
            EXPECT_EQ(lit->functor(), "p");
            EXPECT_EQ(lit->arity(), 0);

            // note: KBp is simplified to Bp
            auto label = (ModalityLabel*)(lit->label().get());
            EXPECT_EQ(label->epistemicOperator(), BeliefModality::B());
            EXPECT_EQ(label->pastOperator().get(), nullptr);
        }
    }
}

TEST_F(QueryTreeTest, Conjunction_pq)
{
    QueryTree qt(p_ & q_);
    EXPECT_EQ(qt.numPaths(), 1);
    if(qt.numPaths()==1) {
        for(auto pathIt = qt.paths().begin(); pathIt != qt.paths().end(); pathIt++) {
            auto path = *pathIt;
            EXPECT_EQ(path.numLiterals(), 2);
            int flags = 0;
            if(path.numLiterals() == 2) {
                for(auto litIt = path.literals().begin(); litIt != path.literals().end(); litIt++) {
                    auto lit = *litIt;
                    EXPECT_FALSE(lit->isNegated());
                    EXPECT_TRUE(*lit->label() == *emptyLabel());
                    if("p" == lit->functor()) {
                        flags |= 1;
                    }
                    if("q" == lit->functor()) {
                        flags |= 2;
                    }
                }
                EXPECT_EQ(flags, 3);
            }
        }
        /*auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 2);
        if(path.numLiterals() == 2) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(!lit->isNegated());
            EXPECT_TRUE(*lit->label() == *emptyLabel());
        }
        */
    }
}

TEST_F(QueryTreeTest, Conjunction_pqr)
{
    QueryTree qt(~p_ & q_ & r_);
    EXPECT_EQ(qt.numPaths(), 1);
    if(qt.numPaths()==1) {
        for(auto pathIt = qt.paths().begin(); pathIt != qt.paths().end(); pathIt++) {
            auto path = *pathIt;
            EXPECT_EQ(path.numLiterals(), 3);
            if(path.numLiterals() == 3) {
                int flags = 0;
                for(auto litIt = path.literals().begin(); litIt != path.literals().end(); litIt++) {
                    auto lit = *litIt;
                    EXPECT_FALSE((lit->isNegated()) && ("p" != lit->functor()));
                    EXPECT_TRUE(*lit->label() == *emptyLabel());
                    if("p" == lit->functor()) {
                        flags |= 1;
                    }
                    if("q" == lit->functor()) {
                        flags |= 2;
                    }
                    if("r" == lit->functor()) {
                        flags |= 4;
                    }
                }
                EXPECT_EQ(flags, 7);
            }
        }
        /*auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 3);
        if(path.numLiterals() == 3) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(lit->isNegated());
            EXPECT_TRUE(*lit->label() == *emptyLabel());
        }
        */
    }
}

TEST_F(QueryTreeTest, Disjunction_pq)
{
    QueryTree qt(p_ | q_);
    EXPECT_EQ(qt.numPaths(), 2);
    if(qt.numPaths()==2) {
        int flags = 0;
        for(auto pathIt = qt.paths().begin(); pathIt != qt.paths().end(); pathIt++) {
            auto path = *pathIt;
            EXPECT_EQ(path.numLiterals(), 1);
            if(path.numLiterals() == 1) {
                for(auto litIt = path.literals().begin(); litIt != path.literals().end(); litIt++) {
                    auto lit = *litIt;
                    EXPECT_FALSE(lit->isNegated());
                    EXPECT_TRUE(*lit->label() == *emptyLabel());
                    if("p" == lit->functor()) {
                        flags |= 1;
                    }
                    if("q" == lit->functor()) {
                        flags |= 2;
                    }
                }
            }
        }
        EXPECT_EQ(flags, 3);
        /*auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 1);
        if(path.numLiterals() == 1) {
            auto &lit = path.literals().front();
            EXPECT_FALSE(lit->isNegated());
            EXPECT_TRUE(*lit->label() == *emptyLabel());
        }
        */
    }
}

TEST_F(QueryTreeTest, Disjunction_pqr)
{
    QueryTree qt(~p_ | q_ | r_);
    EXPECT_EQ(qt.numPaths(), 3);
    if(qt.numPaths()==3) {
        int flags = 0;
        for(auto pathIt = qt.paths().begin(); pathIt != qt.paths().end(); pathIt++) {
            auto path = *pathIt;
            EXPECT_EQ(path.numLiterals(), 1);
            if(path.numLiterals() == 1) {
                for(auto litIt = path.literals().begin(); litIt != path.literals().end(); litIt++) {
                    auto lit = *litIt;
                    EXPECT_FALSE((lit->isNegated()) && ("p" != lit->functor()));
                    EXPECT_TRUE(*lit->label() == *emptyLabel());
                    if("p" == lit->functor()) {
                        flags |= 1;
                    }
                    if("q" == lit->functor()) {
                        flags |= 2;
                    }
                    if("r" == lit->functor()) {
                        flags |= 4;
                    }
                }
            }
        }
        EXPECT_EQ(flags, 7);
        /*auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 1);
        if(path.numLiterals() == 1) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(*lit->label() == *emptyLabel());
        }
        */
    }
}

TEST_F(QueryTreeTest, AndOr)
{
    QueryTree qt((p_ | ~r_) & (q_ | r_));
    EXPECT_EQ(qt.numPaths(), 4);
    if(qt.numPaths()==4) {
        int flags = 0;
        for(auto pathIt = qt.paths().begin(); pathIt != qt.paths().end(); pathIt++) {
            int flagsP = 0;
            auto path = *pathIt;
            EXPECT_EQ(path.numLiterals(), 2);
            if(path.numLiterals() == 2) {
                for(auto litIt = path.literals().begin(); litIt != path.literals().end(); litIt++) {
                    auto lit = *litIt;
                    EXPECT_FALSE((lit->isNegated()) && ("r" != lit->functor()));
                    EXPECT_TRUE(*lit->label() == *emptyLabel());
                    if("p" == lit->functor()) {
                        flagsP |= 1;
                    }
                    if("q" == lit->functor()) {
                        flagsP |= 2;
                    }
                    if(("r" == lit->functor()) && (lit->isNegated())) {
                        flagsP |= 4;
                    }
                    if(("r" == lit->functor()) && (!lit->isNegated())) {
                        flagsP |= 8;
                    }
                }
                if(3 == flagsP) { //pq
                    flags |= 1;
                }
                if(9 == flagsP) { //pr
                    flags |= 2;
                }
                if(6 == flagsP) { //-rq
                    flags |= 4;
                }
                if(12 == flagsP) { //-rr
                    flags |= 8;
                }
            }
        }
        EXPECT_EQ(flags, 15);
        /*auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 2);
        if(path.numLiterals() == 2) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(*lit->label() == *emptyLabel());
        }
        */
    }
}

TEST_F(QueryTreeTest, OrAnd)
{
    QueryTree qt((p_ & ~r_) | (q_ & r_));
    EXPECT_EQ(qt.numPaths(), 2);
    if(qt.numPaths()==2) {
        int flags = 0;
        for(auto pathIt = qt.paths().begin(); pathIt != qt.paths().end(); pathIt++) {
            int flagsP = 0;
            auto path = *pathIt;
            EXPECT_EQ(path.numLiterals(), 2);
            if(path.numLiterals() == 2) {
                for(auto litIt = path.literals().begin(); litIt != path.literals().end(); litIt++) {
                    auto lit = *litIt;
                    EXPECT_FALSE((lit->isNegated()) && ("r" != lit->functor()));
                    EXPECT_TRUE(*lit->label() == *emptyLabel());
                    if("p" == lit->functor()) {
                        flagsP |= 1;
                    }
                    if("q" == lit->functor()) {
                        flagsP |= 2;
                    }
                    if(("r" == lit->functor()) && (lit->isNegated())) {
                        flagsP |= 4;
                    }
                    if(("r" == lit->functor()) && (!lit->isNegated())) {
                        flagsP |= 8;
                    }
                }
                if(5 == flagsP) { //p-r
                    flags |= 1;
                }
                if(10 == flagsP) { //qr
                    flags |= 2;
                }
            }
        }
        EXPECT_EQ(flags, 3);
        /*auto &path = qt.paths().front();
        EXPECT_EQ(path.numLiterals(), 2);
        if(path.numLiterals() == 2) {
            auto &lit = path.literals().front();
            EXPECT_TRUE(*lit->label() == *emptyLabel());
        }
        */
    }
}

