/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Top.h"

using namespace knowrob;

const std::shared_ptr<TopTerm>& TopTerm::get()
{
	static std::shared_ptr<TopTerm> singleton(new TopTerm);
	return singleton;
}

TopTerm::TopTerm()
: Predicate("true", std::vector<TermPtr>())
{
}

void TopTerm::write(std::ostream& os) const
{
	os << "\u22A4";
}
