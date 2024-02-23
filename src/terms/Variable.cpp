/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>
#include <gtest/gtest.h>
#include "knowrob/terms/Variable.h"

using namespace knowrob;

Variable::Variable(std::string_view name)
		: Term(),
		  name_(name),
		  variables_({this}) {
}

bool Variable::isSameVariable(const Variable &other) const {
	return name_ == other.name_;
}

bool Variable::operator<(const Variable &other) const {
	return (this->name_ < other.name_);
}

void Variable::write(std::ostream &os) const {
	os << name_;
}
