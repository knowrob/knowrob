//
// Created by danielb on 02.08.23.
//

#include "knowrob/formulas/Literal.h"

using namespace knowrob;

std::ostream& Literal::write(std::ostream& os) const
{
    if(isNegative()) {
        os << "not(" << *predicate_ << ")";
    }
    else {
        os << *predicate_;
    }
    return os;
}

std::ostream& LabeledLiteral::write(std::ostream& os) const
{
    // TODO: also print modal frame
    if(isNegative()) {
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
