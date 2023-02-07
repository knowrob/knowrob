/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/terms/Bottom.h"

using namespace knowrob;

const std::shared_ptr<BottomTerm>& BottomTerm::get()
{
	static std::shared_ptr<BottomTerm> singleton(new BottomTerm);
	return singleton;
}

BottomTerm::BottomTerm()
: Predicate("false", std::vector<TermPtr>())
{
}

void BottomTerm::write(std::ostream& os) const
{
	os << "\u22A5";
}

TEST(bottom_term, isGround) {
    EXPECT_TRUE(BottomTerm::get()->isGround());
}
