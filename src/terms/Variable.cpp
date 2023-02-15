/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <utility>
// GTEST
#include <gtest/gtest.h>
// KnowRob
#include "knowrob/Logger.h"
#include "knowrob/terms/Variable.h"

using namespace knowrob;

Variable::Variable(std::string name)
: Term(TermType::VARIABLE),
  name_(std::move(name)),
  variables_({ this })
{
}

bool Variable::isEqual(const Term& other) const
{
    return name_ == static_cast<const Variable&>(other).name_; // NOLINT
}

bool Variable::operator< (const Variable& other) const
{
	return (this->name_ < other.name_);
}

void Variable::write(std::ostream& os) const
{
	os << name_;
}
