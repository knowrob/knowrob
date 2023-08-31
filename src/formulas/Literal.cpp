//
// Created by danielb on 02.08.23.
//

#include "knowrob/formulas/Literal.h"

using namespace knowrob;

Literal::Literal(const PredicatePtr &predicate, bool isNegative, const ModalityLabelPtr &label)
: predicate_(predicate),
  isNegated_(isNegative),
  label_(label)
{}

Literal::Literal(const Literal &other, const Substitution &sub)
: predicate_(std::make_shared<Predicate>(*other.predicate_, sub)),
  isNegated_(other.isNegated_),
  label_(other.label_)
{}

std::ostream& Literal::write(std::ostream& os) const
{
    // TODO: also print label
    if(isNegated()) {
        os << "not(" << *predicate_ << ")";
    }
    else {
        os << *predicate_;
    }
    return os;
}

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::Literal& l)
    { return l.write(os); }
}
