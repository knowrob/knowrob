/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Formula.h>

using namespace knowrob;

Formula::Formula(const FormulaType &type)
: type_(type)
{}

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Formula& phi) //NOLINT
	{
		phi.write(os);
		return os;
	}
}
