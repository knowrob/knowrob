//
// Created by daniel on 06.03.23.
//

#include "knowrob/Statement.h"

using namespace knowrob;

Statement::Statement(const PredicatePtr &predicate)
: predicate_(predicate),
  o_timeInterval_(std::nullopt),
  o_confidenceValue_(std::nullopt)
{
}

void Statement::setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval)
{
    timeInterval_ = timeInterval;
    o_timeInterval_ = (timeInterval_ ?
                       std::optional<const TimeInterval*>(timeInterval_.get()) :
                       std::optional<const TimeInterval*>(std::nullopt));
}

void Statement::setConfidenceValue(const std::shared_ptr<ConfidenceValue> &confidenceValue)
{
    confidenceValue_ = confidenceValue;
    o_confidenceValue_ = (confidenceValue_ ?
                             std::optional<const ConfidenceValue*>(confidenceValue_.get()) :
                             std::optional<const ConfidenceValue*>(std::nullopt));
}

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::Statement& s) //NOLINT
    {
        os << *s.predicate();
        return os;
    }
}