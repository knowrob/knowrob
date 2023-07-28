/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/ModalQuery.h>
#include <knowrob/formulas/Predicate.h>

using namespace knowrob;

ModalQuery::ModalQuery(const std::shared_ptr<Formula> &formula, int flags)
: Query(flags), formula_(formula)
{
}

std::shared_ptr<ModalQuery> ModalQuery::applySubstitution(const Substitution &sub) const
{
	return std::make_shared<ModalQuery>(
		formula_->isGround() ?
		formula_ :
		formula_->applySubstitution(sub),
		flags_
	);
}

const ModalityFrame& ModalQuery::modalFrame() const
{
	static ModalityFrame empty;
    return empty;
}

std::ostream& ModalQuery::print(std::ostream &os) const
{
    os << *formula();
    return os;
}
