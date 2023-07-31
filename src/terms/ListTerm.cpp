/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/Constant.h"

using namespace knowrob;

ListTerm::ListTerm(const std::vector<TermPtr> &elements)
: Term(TermType::LIST),
  elements_(elements),
  variables_(getVariables1())
{
}

bool ListTerm::isEqual(const Term& other) const {
	const auto &x = static_cast<const ListTerm&>(other); // NOLINT
    for(int i=0; i<elements_.size(); ++i) {
        if(!(*(elements_[i]) == *(x.elements_[i]))) return false;
    }
    return true;
}

VariableSet ListTerm::getVariables1() const
{
	VariableSet out;
	for(auto &arg : elements_) {
		for(auto &var : arg->getVariables()) {
			out.insert(var);
		}
	}
	return out;
}
		
std::shared_ptr<ListTerm> ListTerm::nil()
{
	static std::shared_ptr<ListTerm> x(new ListTerm(
		std::vector<std::shared_ptr<Term>>(0)));
	return x;
}

bool ListTerm::isNIL() const
{
	return elements_.empty();
}

size_t ListTerm::computeHash() const
{
    static const auto GOLDEN_RATIO_HASH = static_cast<size_t>(0x9e3779b9);
    auto seed = static_cast<size_t>(0);

    for(const auto &item : elements_) {
        /* Combine the hashes.
           The function (a ^ (b + GOLDEN_RATIO_HASH + (a << 6) + (a >> 2))) is known to
           give a good distribution of hash values across the range of size_t. */
        seed ^= item->computeHash() + GOLDEN_RATIO_HASH + (seed << 6) + (seed >> 2);
    }

    return seed;
}

void ListTerm::write(std::ostream& os) const
{
	os << '[';
	for(uint32_t i=0; i<elements_.size(); i++) {
		Term *t = elements_[i].get();
		os << (*t);
		if(i+1 < elements_.size()) {
			os << ',' << ' ';
		}
	}
	os << ']';
}

/******************************************/
/************** Unit Tests ****************/
/******************************************/

TEST(list_term, NIL) {
	EXPECT_TRUE(ListTerm::nil()->elements().empty());
	EXPECT_TRUE(ListTerm({}).elements().empty());
	EXPECT_TRUE(ListTerm({}).isNIL());
	EXPECT_TRUE(ListTerm::nil()->isAtomic());
	EXPECT_TRUE(ListTerm::nil()->isGround());
}

TEST(list_term, lists) {
    auto x = std::make_shared<StringTerm>("x");
    auto y = std::make_shared<StringTerm>("y");
    EXPECT_TRUE(ListTerm({x}).isGround());
    EXPECT_FALSE(ListTerm({x}).isAtomic());
    EXPECT_FALSE(ListTerm({x}).elements().empty());
    EXPECT_EQ(ListTerm({x,y}).elements()[0], x);
    EXPECT_EQ(ListTerm({x,y}).elements()[1], y);
}
