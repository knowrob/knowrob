/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/String.h"

using namespace knowrob;

ListTerm::ListTerm(const std::vector<TermPtr> &elements)
		: Function(listFunctor(), elements) {
}

const AtomPtr &ListTerm::listFunctor() {
	static const AtomPtr fun = Atom::Tabled("[]");
	return fun;
}

std::shared_ptr<ListTerm> ListTerm::nil() {
	static std::shared_ptr<ListTerm> x(new ListTerm(
			std::vector<std::shared_ptr<Term>>(0)));
	return x;
}

bool ListTerm::isNIL() const {
	return arguments_.empty();
}

void ListTerm::write(std::ostream &os) const {
	os << '[';
	for (uint32_t i = 0; i < arguments_.size(); i++) {
		os << *arguments_[i];
		if (i + 1 < arguments_.size()) {
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
	auto x = std::make_shared<String>("x");
	auto y = std::make_shared<String>("y");
	EXPECT_TRUE(ListTerm({x}).isGround());
	EXPECT_FALSE(ListTerm({x}).isAtomic());
	EXPECT_FALSE(ListTerm({x}).elements().empty());
	EXPECT_EQ(ListTerm({x, y}).elements()[0], x);
	EXPECT_EQ(ListTerm({x, y}).elements()[1], y);
}
