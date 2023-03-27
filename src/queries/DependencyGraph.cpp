/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>

#include "knowrob/queries/DependencyGraph.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/modalities/KnowledgeModality.h"

using namespace knowrob;

static inline bool has_intersection(const VariableSet &a, const VariableSet &b)
{
    // TODO: is there a more standard way of doing this?
    const VariableSet &smaller = (a.size()<b.size() ? a : b);
    const VariableSet &larger = ((&smaller == &a) ? b : a);
    return std::any_of(smaller.cbegin(), smaller.cend(),
                       [larger](auto *v){ return larger.count(v)>0; });
}

LiteralDependencyNode::LiteralDependencyNode(const LiteralPtr &literal)
        : DependencyNode(),
          literal_(literal)
{
}

ModalDependencyNode::ModalDependencyNode(
        const ModalityLabelPtr &label, const std::list<LiteralPtr> &literals)
        : DependencyNode(),
          label_(label)
{
    for(auto lit : literals) {
        auto &litVars = lit->predicate()->getVariables();
        // add a new node
        auto &litNode = nodes_.emplace_back(lit);
        // find the literal node groups that have a shared variable with the literal.
        std::list<std::list<DependencyGroup < LiteralDependencyNode>>::iterator > dependencies;
        for(auto it=groups_.begin(); it!=groups_.end(); ++it) {
            if(has_intersection(it->variables_, litVars)) {
                dependencies.push_back(it);
            }
        }

        if(dependencies.empty()) {
            // the literal does not have a shared variable with an existing group.
            // create a new one with just the literal as a member.
            auto &newGroup = groups_.emplace_back();
            newGroup.variables_ = litVars;
            newGroup.member_.push_back(&litNode);
        }
        else if(dependencies.size()==1) {
            // add to group
            auto &group = dependencies.front();
            group->member_.push_back(&litNode);
            group->variables_.insert(litVars.begin(), litVars.end());
        }
        else {
            auto &newGroup = groups_.emplace_back();
            for(auto groupIterator : dependencies) {
                newGroup += *groupIterator;
                groups_.erase(groupIterator);
            }
        }

        // remember variables that appear in the literal
        variables_.insert(litVars.begin(), litVars.end());
    }
}

unsigned long DependencyGraph::numLiteralNodes() const
{
    unsigned long count=0;
    for(auto &n : nodes_) {
        count += n.numLiteralNodes();
    }
    return count;
}

unsigned long DependencyGraph::numLiteralGroups() const
{
    unsigned long count=0;
    for(auto &n : nodes_) {
        count += n.numLiteralGroups();
    }
    return count;
}

void DependencyGraph::addNodes(const ModalityLabelPtr &label,
                               const std::list<LiteralPtr> &literals)
{
    auto &newNode = nodes_.emplace_back(label, literals);

    // find the modal node groups that have a shared variable with newNode.
    std::list<std::list<DependencyGroup < ModalDependencyNode>>::iterator > dependencies;
    for(auto it=groups_.begin(); it!=groups_.end(); ++it) {
        if(has_intersection(it->variables_, newNode.variables())) {
            dependencies.push_back(it);
        }
    }

    DependencyGroup<ModalDependencyNode> *newNodeGroup = nullptr;
    if(dependencies.empty()) {
        // the literal does not have a shared variable with an existing group.
        // create a new one with just the literal as a member.
        auto &newGroup = groups_.emplace_back();
        newNodeGroup = &newGroup;
    }
    else if(dependencies.size()==1) {
        // add to group
        auto &group = *dependencies.front();
        newNodeGroup = &group;
    }
    else {
        // merge multiple groups into one
        auto &newGroup = groups_.emplace_back();
        for(auto groupIterator : dependencies) {
            newGroup += *groupIterator;
            groups_.erase(groupIterator);
        }
        newNodeGroup = &newGroup;
    }
    // update neighbor relation based on shared variables with other nodes
    for(auto &x : newNodeGroup->member_) {
        if(has_intersection(x->variables(), newNode.variables())) {
            x->neighbors_.push_back(&newNode);
            newNode.neighbors_.push_back(x);
        }
    }
    // finally add the new node to the group
    newNodeGroup->member_.push_back(&newNode);
    newNodeGroup->variables_.insert(newNode.variables().begin(), newNode.variables().end());
}


// fixture class for testing
class DependencyGraphTest : public ::testing::Test {
protected:
    LiteralPtr p_, q_, r_, s_;
    ModalityLabelPtr m1_, m2_;
    void SetUp() override {
        p_ = std::make_shared<Literal>(QueryParser::parsePredicate("p(a,X)"), false);
        q_ = std::make_shared<Literal>(QueryParser::parsePredicate("q(X,Y)"), false);
        r_ = std::make_shared<Literal>(QueryParser::parsePredicate("r(Y,z)"), false);
        s_ = std::make_shared<Literal>(QueryParser::parsePredicate("s(b,a)"), false);

        auto x = std::make_shared<ModalIteration>();
        *x += KnowledgeModality::K();
        m2_ = std::make_shared<ModalityLabel>(x);
    }
    void TearDown() override {}
};

TEST_F(DependencyGraphTest, SingleLiteral)
{
    DependencyGraph dg;
    dg.addNodes(m1_, { p_ });
    ASSERT_EQ(dg.numModalNodes(),1);
    ASSERT_EQ(dg.numModalGroups(),1);
    ASSERT_EQ(dg.numLiteralNodes(),1);
    ASSERT_EQ(dg.numLiteralGroups(),1);
}

TEST_F(DependencyGraphTest, DependantLiterals)
{
    DependencyGraph dg;
    dg.addNodes(m2_, { p_, q_ });
    ASSERT_EQ(dg.numModalNodes(),1);
    ASSERT_EQ(dg.numModalGroups(),1);
    ASSERT_EQ(dg.numLiteralNodes(),2);
    ASSERT_EQ(dg.numLiteralGroups(),1);
}

TEST_F(DependencyGraphTest, IndependantLiterals)
{
    DependencyGraph dg;
    dg.addNodes(m1_, { p_, r_ });
    ASSERT_EQ(dg.numModalNodes(),1);
    ASSERT_EQ(dg.numModalGroups(),1);
    ASSERT_EQ(dg.numLiteralNodes(),2);
    ASSERT_EQ(dg.numLiteralGroups(),2);
}

TEST_F(DependencyGraphTest, ChainAndOne)
{
    DependencyGraph dg;
    dg.addNodes(m1_, { p_, q_, r_, s_ });
    ASSERT_EQ(dg.numModalNodes(),1);
    ASSERT_EQ(dg.numModalGroups(),1);
    ASSERT_EQ(dg.numLiteralNodes(),4);
    ASSERT_EQ(dg.numLiteralGroups(),2);
}

TEST_F(DependencyGraphTest, MultiModalDependant)
{
    DependencyGraph dg;
    dg.addNodes(m1_, { p_, r_, s_ });
    dg.addNodes(m2_, { q_ });
    ASSERT_EQ(dg.numModalNodes(),2);
    ASSERT_EQ(dg.numModalGroups(),1);
    ASSERT_EQ(dg.numLiteralNodes(),4);
    ASSERT_EQ(dg.numLiteralGroups(),4);
}

TEST_F(DependencyGraphTest, MultiModalIndependant)
{
    DependencyGraph dg;
    dg.addNodes(m1_, { p_, r_, q_ });
    dg.addNodes(m2_, { s_ });
    ASSERT_EQ(dg.numModalNodes(),2);
    ASSERT_EQ(dg.numModalGroups(),2);
    ASSERT_EQ(dg.numLiteralNodes(),4);
    ASSERT_EQ(dg.numLiteralGroups(),2);
}
