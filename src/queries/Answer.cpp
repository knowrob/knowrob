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
: substitution_(std::make_shared<Substitution>())
{
}

Answer::Answer(const Answer &other)
: substitution_(std::make_shared<Substitution>(*other.substitution_)),
  predicates_(other.predicates_),
  modality_(other.modality_)
{
}

const std::shared_ptr<const Answer>& Answer::emptyResult()
{
	static auto result = std::make_shared<const Answer>();
	return result;
}

void Answer::substitute(const Variable &var, const TermPtr &term)
{
	substitution_->set(var, term);
}

bool Answer::hasSubstitution(const Variable &var)
{
	return substitution_->contains(var);
}

void Answer::addPredicate(const std::shared_ptr<StringTerm> &reasonerModule,
                          const std::shared_ptr<Predicate> &predicate)
{
	predicates_.emplace_back(reasonerModule, predicate);
}

bool Answer::combine(const std::shared_ptr<const Answer> &other, Reversible *changes)
{
	// unify substitutions
	if(!substitution_->unifyWith(*other->substitution_, changes)) {
		// unification failed -> results cannot be combined
		return false;
	}
	// compute intersection of time intervals
	if(!combineModalFrame(other)) {
		return false;
	}
	// merge instantiated predicates
	predicates_.insert(predicates_.end(),
					   other->predicates_.begin(),
					   other->predicates_.end());
	return true;
}

bool Answer::combineModalFrame(const std::shared_ptr<const Answer> &other)
{
	// TODO: think about how modal frame of answers could be combined!
	//modality_ = modality_.combine(other->modality_);
	KB_WARN("todo: implement combining modal frames.");
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Answer& solution) //NOLINT
	{
		if(solution.modalFrame().hasValue()) {
			os << solution.modalFrame() << "::";
		}

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