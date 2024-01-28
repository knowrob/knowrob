/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/formulas/Bottom.h"

using namespace knowrob;

const std::shared_ptr<Bottom>& Bottom::get()
{
	static std::shared_ptr<Bottom> singleton(new Bottom);
	return singleton;
}

Bottom::Bottom()
: Predicate("false", std::vector<TermPtr>())
{
}

void Bottom::write(std::ostream& os) const
{
	os << "\u22A5";
}

bool Bottom::isEqual(const Formula &other) const
{
	return true; // isEqual is only called of other also has type "Bottom"
}

TEST(bottom_term, isGround) {
    EXPECT_TRUE(Bottom::get()->isGround());
}
