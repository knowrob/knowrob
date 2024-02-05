/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/terms/Term.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/formulas/Top.h"

using namespace knowrob;

const VariableSet Term::noVariables_ = {};

Term::Term(TermType type)
: type_(type)
{}

bool Term::operator==(const Term& other) const
{
	// note: isEqual can safely perform static cast as type id's do match
	return typeid(*this) == typeid(other) && isEqual(other);
}

std::string Term::toString() const
{
	std::stringstream ss;
	write(ss);
	return ss.str();
}

bool VariableComparator::operator()(const Variable* const &v0, const Variable* const &v1) const
{
	return *v0 < *v1;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const Term& t) //NOLINT
	{
		t.write(os);
		return os;
	}
}
