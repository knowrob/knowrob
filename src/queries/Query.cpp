/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/Query.h>
#include <knowrob/formulas/Predicate.h>

using namespace knowrob;

Query::Query(const std::shared_ptr<Formula> &formula, int flags, ModalFrame modalFrame)
: formula_(formula), flags_(flags), modalFrame_(std::move(modalFrame))
{
}

std::shared_ptr<Query> Query::applySubstitution(const Substitution &sub) const
{
	return std::make_shared<Query>(
		formula_->isGround() ?
		formula_ :
		formula_->applySubstitution(sub),
		flags_
	);
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q) //NOLINT
	{
	    // TODO: include modal frame in ostream
		os << *q.formula();
		return os;
	}
}
