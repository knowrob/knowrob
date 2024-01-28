/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/Top.h"

using namespace knowrob;

const std::shared_ptr<Top>& Top::get()
{
	static std::shared_ptr<Top> singleton(new Top);
	return singleton;
}

Top::Top()
: Predicate("true", std::vector<TermPtr>())
{
}

bool Top::isEqual(const Formula &other) const
{
	return true; // isEqual is only called of other also has type "Bottom"
}

void Top::write(std::ostream& os) const
{
	os << "\u22A4";
}
