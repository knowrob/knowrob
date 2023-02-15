/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/QueryResult.h>

using namespace knowrob;

QueryResult::QueryResult()
: substitution_(std::make_shared<Substitution>()),
  o_timeInterval_(std::nullopt),
  o_confidence_(std::nullopt)
{
}

QueryResult::QueryResult(const QueryResult &other)
: substitution_(std::make_shared<Substitution>(*other.substitution_)),
  predicates_(other.predicates_)
{
	setTimeInterval(other.timeInterval_);
	setConfidenceValue(other.confidence_);
}

const std::shared_ptr<const QueryResult>& QueryResult::emptyResult()
{
	static auto result = std::make_shared<const QueryResult>();
	return result;
}

void QueryResult::substitute(const Variable &var, const TermPtr &term)
{
	substitution_->set(var, term);
}

bool QueryResult::hasSubstitution(const Variable &var)
{
	return substitution_->contains(var);
}

void QueryResult::addPredicate(const std::shared_ptr<StringTerm> &reasonerModule,
							   const std::shared_ptr<Predicate> &predicate)
{
	predicates_.emplace_back(reasonerModule, predicate);
}

void QueryResult::setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval)
{
	timeInterval_ = timeInterval;
	o_timeInterval_ = (timeInterval_ ?
			std::optional<const TimeInterval*>(timeInterval_.get()) :
			std::optional<const TimeInterval*>(std::nullopt));
}

void QueryResult::setConfidenceValue(const std::shared_ptr<ConfidenceValue> &confidence)
{
	confidence_ = confidence;
	o_confidence_ = (confidence_ ?
			std::optional<const ConfidenceValue*>(confidence_.get()) :
			std::optional<const ConfidenceValue*>(std::nullopt));
}

bool QueryResult::combine(const std::shared_ptr<const QueryResult> &other, Reversible *changes)
{
	// unify substitutions
	if(!substitution_->unifyWith(*other->substitution_, changes)) {
		// unification failed -> results cannot be combined
		return false;
	}
	// compute intersection of time intervals
	if(combineTimeInterval(other->timeInterval_, changes)) {
		// intersection empty -> results cannot be combined
		if(timeInterval_->empty()) return false;
	}
	// accumulate confidence values
	auto oldConfidence = confidence_;
	if(combineConfidence(other->confidence_)) {
		if(changes) changes->push([this,oldConfidence](){
			setConfidenceValue(oldConfidence);
		});
	}
	// merge instantiated predicates
	predicates_.insert(predicates_.end(),
					   other->predicates_.begin(),
					   other->predicates_.end());
	return true;
}

bool QueryResult::combineTimeInterval(const std::shared_ptr<TimeInterval> &otherTimeInterval, Reversible *reversible)
{
	if(otherTimeInterval) {
		if(o_timeInterval_.has_value()) {
			if(!otherTimeInterval->isMoreGeneralThan(*timeInterval_)) {
				auto old = timeInterval_;
				setTimeInterval(timeInterval_->intersectWith(*otherTimeInterval));
				if(reversible) reversible->push([this,old](){ setTimeInterval(old); });
				return true;
			}
		}
		else {
			setTimeInterval(otherTimeInterval);
			if(reversible) reversible->push([this](){ setTimeInterval(std::shared_ptr<TimeInterval>()); });
			return true;
		}
	}
	return false;
}

bool QueryResult::combineConfidence(const std::shared_ptr<ConfidenceValue> &otherConfidence)
{
	if(otherConfidence) {
		if(confidence_) {
			// FIXME: this is not ok if there is dependency between subgoals
			auto newConfidence = otherConfidence->value() * confidence_->value();
			setConfidenceValue(std::make_shared<ConfidenceValue>(newConfidence));
			return true;
		}
		else {
			setConfidenceValue(otherConfidence);
			return true;
		}
	}
	else {
		return false;
	}
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::QueryResult& solution) //NOLINT
	{
		if(solution.confidence().has_value()) {
			os << *solution.confidence().value() << "::";
		}
		if(solution.timeInterval().has_value()) {
			os << *solution.timeInterval().value() << "::";
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