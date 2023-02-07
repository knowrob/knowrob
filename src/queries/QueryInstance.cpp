/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/QueryInstance.h>

using namespace knowrob;

QueryInstance::QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
							 const std::shared_ptr<QueryResultStream::Channel> &outputChannel,
							 const std::shared_ptr<const QueryResult> &partialResult)
: uninstantiatedQuery_(uninstantiatedQuery),
  outputChannel_(outputChannel),
  partialResult_(partialResult)
{
}

QueryInstance::QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
							 const std::shared_ptr<QueryResultStream::Channel> &outputChannel)
: uninstantiatedQuery_(uninstantiatedQuery),
  outputChannel_(outputChannel),
  partialResult_(QueryResult::emptyResult())
{
}

std::shared_ptr<const Query> QueryInstance::create()
{
	// TODO: set scope for query generated, else would be dangerous because
	//   both QueryInstance and Query have an interface for getting the scope.
	//   maybe just hide the query object? seems query instance might be sufficient
	if(partialResult_->substitution()->empty()) {
		return uninstantiatedQuery_;
	}
	else {
		return uninstantiatedQuery_->applySubstitution(*partialResult_->substitution());
	}
}

void QueryInstance::pushSolution(const std::shared_ptr<QueryResult> &solution)
{
	// include substitutions of partialResult_
	for (const auto& pair: *partialResult_->substitution())
		if (!solution->hasSubstitution(pair.first)) {
			solution->substitute(pair.first, pair.second);
		} else {
			// assume both terms unify
		}

	// include instantiations of partialResult_
	for (const auto& pi: partialResult_->predicates())
		solution->addPredicate(pi.reasonerModule(), pi.predicate());

	// combine time intervals
	std::shared_ptr<TimeInterval> otherTimeInterval = (
			partialResult_->timeInterval().has_value() ?
			partialResult_->timeInterval_ :
			uninstantiatedQuery_->timeInterval_);
	solution->combineTimeInterval(otherTimeInterval);
	// ignore solution if time interval is empty
	if(solution->timeInterval_ && solution->timeInterval_->empty()) return;

	// combine confidence values
	solution->combineConfidence(partialResult_->confidence_);
	// ignore solution if confidence value does not lie within desired interval
	if(uninstantiatedQuery_->confidenceInterval_ && solution->confidence_ &&
	  !uninstantiatedQuery_->confidenceInterval_->contains(*solution->confidence_)) {
		return;
	}

	// finally push result into the stream
	outputChannel_->push(solution);
}

void QueryInstance::pushEOS()
{
	outputChannel_->push(QueryResultStream::eos());
}

const std::optional<const TimeInterval*>& QueryInstance::timeInterval() const
{
	if(partialResult_->timeInterval().has_value()) {
		return partialResult_->timeInterval();
	}
	else {
		return uninstantiatedQuery_->timeInterval();
	}
}

const std::optional<const ConfidenceInterval*>& QueryInstance::confidenceInterval() const
{
	return uninstantiatedQuery_->confidenceInterval();
}
