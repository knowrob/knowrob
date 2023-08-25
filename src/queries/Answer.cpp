/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/Answer.h>
#include "knowrob/Logger.h"

using namespace knowrob;

Answer::Answer()
: substitution_(std::make_shared<Substitution>()),
  isUncertain_(false),
  timeInterval_(std::nullopt)
{
}

Answer::Answer(const Answer &other)
: substitution_(std::make_shared<Substitution>(*other.substitution_)),
  predicates_(other.predicates_),
  isUncertain_(other.isUncertain_),
  timeInterval_(other.timeInterval_)
{
}

const std::shared_ptr<const Answer>& Answer::emptyAnswer()
{
	static auto result = std::make_shared<const Answer>();
	return result;
}

void Answer::substitute(const Variable &var, const TermPtr &term)
{
	substitution_->set(var, term);
}

bool Answer::hasSubstitution(const Variable &var) const
{
	return substitution_->contains(var);
}

void Answer::addPredicate(const std::shared_ptr<StringTerm> &reasonerModule,
                          const std::shared_ptr<Predicate> &predicate)
{
	predicates_.emplace_back(reasonerModule, predicate);
}

size_t Answer::computeHash() const
{
    return substitution_->computeHash();
}

bool Answer::combine(const std::shared_ptr<const Answer> &other, Reversible *changes)
{
	// unify substitutions
	if(!substitution_->unifyWith(*other->substitution_, changes)) {
		// unification failed -> results cannot be combined
		return false;
	}
	// combine modal frames
	if(other->isUncertain()) isUncertain_ = true;
    if(other->timeInterval().has_value()) {
        if(timeInterval_.has_value()) {
            timeInterval_->intersectWith(other->timeInterval().value());
        } else {
            timeInterval_ = other->timeInterval();
        }
    }

	// merge instantiated predicates
	predicates_.insert(predicates_.end(),
					   other->predicates_.begin(),
					   other->predicates_.end());
	return true;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Answer& solution) //NOLINT
	{
        os << (solution.isCertain() ? "K" : "B") << "::";
        if(solution.timeInterval().has_value())
            os << solution.timeInterval().value();
        os << "::";

		if(solution.substitution()->empty())
			os << "yes";
		else
			os << *solution.substitution();

		if(!solution.predicates().empty()) {
			os << "\nwith:";
			for(auto &instance : solution.predicates()) {
				os << '\n' << '\t' << instance.reasonerModule()->value() << ':' << *instance.predicate();
			}
		}
		return os;
	}
}
