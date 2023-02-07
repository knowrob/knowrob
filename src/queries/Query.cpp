/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/Query.h>
#include <knowrob/formulas/PredicateFormula.h>

using namespace knowrob;

Query::Query(const std::shared_ptr<Formula> &formula)
: formula_(formula),
  o_timeInterval_(std::nullopt),
  o_confidenceInterval_(std::nullopt)
{
}

Query::Query(const std::shared_ptr<Predicate> &predicate)
: formula_(new PredicateFormula(predicate)),
  o_timeInterval_(std::nullopt),
  o_confidenceInterval_(std::nullopt)
{
}

std::shared_ptr<Query> Query::applySubstitution(const Substitution &sub) const
{
	return std::make_shared<Query>(
		formula_->isGround() ?
		formula_ :
		formula_->applySubstitution(sub)
	);
}

void Query::setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval)
{
	timeInterval_ = timeInterval;
	o_timeInterval_ = (timeInterval_ ?
					   std::optional<const TimeInterval*>(timeInterval_.get()) :
					   std::optional<const TimeInterval*>(std::nullopt));
}

void Query::setConfidenceInterval(const std::shared_ptr<ConfidenceInterval> &confidenceInterval)
{
	confidenceInterval_ = confidenceInterval;
	o_confidenceInterval_ = (confidenceInterval_ ?
					 std::optional<const ConfidenceInterval*>(confidenceInterval_.get()) :
					 std::optional<const ConfidenceInterval*>(std::nullopt));
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q) //NOLINT
	{
		os << *q.formula();
		return os;
	}
}
