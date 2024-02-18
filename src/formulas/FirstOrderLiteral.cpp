/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/FirstOrderLiteral.h"

using namespace knowrob;

FirstOrderLiteral::FirstOrderLiteral(const PredicatePtr &predicate, bool isNegative)
: predicate_(predicate),
  isNegated_(isNegative)
{}

FirstOrderLiteral::FirstOrderLiteral(const FirstOrderLiteral &other, const Substitution &sub)
: predicate_(std::make_shared<Predicate>(*other.predicate_, sub)),
  isNegated_(other.isNegated_)
{}

std::ostream& FirstOrderLiteral::write(std::ostream& os) const
{
    if(isNegated()) {
        os << "not(" << *predicate_ << ")";
    }
    else {
        os << *predicate_;
    }
    return os;
}

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::FirstOrderLiteral& l)
    { return l.write(os); }
}
