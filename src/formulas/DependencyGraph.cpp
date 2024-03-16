/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>

#include "knowrob/formulas/DependencyGraph.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/modalities/KnowledgeModality.h"

using namespace knowrob;

static inline bool has_intersection(const std::set<std::string_view> &a, const std::set<std::string_view> &b)
{
    // TODO: is there a more standard way of doing this?
    const std::set<std::string_view> &smaller = (a.size()<b.size() ? a : b);
    const std::set<std::string_view> &larger = ((&smaller == &a) ? b : a);
    return std::any_of(smaller.cbegin(), smaller.cend(),
                       [larger](auto &v){ return larger.count(v)>0; });
}

void DependencyGraph::operator+=(const DependencyNodePtr &node)
{
    insert(node);
}

void DependencyGraph::insert(const std::vector<FirstOrderLiteralPtr> &literals)
{
    for(auto &l : literals)
        insert(std::make_shared<DependencyNode>(l));
}

void DependencyGraph::insert(const FirstOrderLiteralPtr &literal)
{
    insert(std::make_shared<DependencyNode>(literal));
}

void DependencyGraph::insert(const DependencyNodePtr &newNode)
{
    nodes_.push_back(newNode);

    // find the modal node groups that have a shared variable with newNode.
    std::list<std::list<DependencyGroup>::iterator > dependencies;
    for(auto it=groups_.begin(); it!=groups_.end(); ++it) {
        if(has_intersection(it->variables_, newNode->variables())) {
            dependencies.push_back(it);
        }
    }

    DependencyGroup *newNodeGroup = nullptr;
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
        if(has_intersection(x->variables(), newNode->variables())) {
            x->neighbors_.push_back(newNode);
            newNode->neighbors_.push_back(x);
        }
    }
    // finally add the new node to the group
    newNodeGroup->member_.push_back(newNode);
    newNodeGroup->variables_.insert(newNode->variables().begin(), newNode->variables().end());
}


DependencyNode::DependencyNode(const FirstOrderLiteralPtr &literal)
        : literal_(literal)
{
}

void DependencyNode::addDependency(const std::shared_ptr<DependencyNode> &other)
{
    neighbors_.push_back(other);
}


// fixture class for testing
namespace knowrob::testing {
	class DependencyGraphTest : public ::testing::Test {
	protected:
		FirstOrderLiteralPtr p_, q_, r_, s_;

		void SetUp() override {
			p_ = std::make_shared<FirstOrderLiteral>(QueryParser::parsePredicate("p(a,X)"), false);
			q_ = std::make_shared<FirstOrderLiteral>(QueryParser::parsePredicate("q(X,Y)"), false);
			r_ = std::make_shared<FirstOrderLiteral>(QueryParser::parsePredicate("r(Y,z)"), false);
			s_ = std::make_shared<FirstOrderLiteral>(QueryParser::parsePredicate("s(b,a)"), false);

			auto x = std::make_shared<ModalIteration>();
			*x += KnowledgeModality::K();
		}

		void TearDown() override {}
	};
}
using namespace knowrob::testing;

TEST_F(DependencyGraphTest, SingleLiteral)
{
    DependencyGraph dg;
    dg.insert({p_});
    ASSERT_EQ(dg.numNodes(),1);
    ASSERT_EQ(dg.numGroups(),1);
}

TEST_F(DependencyGraphTest, DependantLiterals)
{
    DependencyGraph dg;
    dg.insert({p_, q_});
    ASSERT_EQ(dg.numNodes(),2);
    ASSERT_EQ(dg.numGroups(),1);
}

TEST_F(DependencyGraphTest, IndependantLiterals)
{
    DependencyGraph dg;
    dg.insert({p_, r_});
    ASSERT_EQ(dg.numNodes(),2);
    ASSERT_EQ(dg.numGroups(),2);
}

TEST_F(DependencyGraphTest, ChainAndOne)
{
    DependencyGraph dg;
    dg.insert({p_, q_, r_});
    ASSERT_EQ(dg.numNodes(),3);
    ASSERT_EQ(dg.numGroups(),1);
}

TEST_F(DependencyGraphTest, MultiModalDependant)
{
    DependencyGraph dg;
    dg.insert({p_, r_, s_});
    dg.insert({q_});
    ASSERT_EQ(dg.numNodes(),4);
    ASSERT_EQ(dg.numGroups(),2);
}

TEST_F(DependencyGraphTest, MultiModalIndependant)
{
    DependencyGraph dg;
    dg.insert({p_, r_, q_});
    dg.insert({s_});
    ASSERT_EQ(dg.numNodes(),4);
    ASSERT_EQ(dg.numGroups(),2);
}
