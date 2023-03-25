/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Formula.h>
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Bottom.h"

using namespace knowrob;

Formula::Formula(const FormulaType &type)
: type_(type)
{}

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

bool Formula::isBottom() const
{
    return (this == Bottom::get().get());
}

bool Formula::isTop() const
{
    return (this == Top::get().get());
}

bool FormulaLabel::operator==(const FormulaLabel &other)
{
    return typeid(*this) == typeid(other) && isEqual(other);
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Formula& phi) //NOLINT
	{
		phi.write(os);
		return os;
	}
}
