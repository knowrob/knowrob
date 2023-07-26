//
// Created by daniel on 06.03.23.
//

#include "knowrob/queries/AtomicProposition.h"

using namespace knowrob;

AtomicProposition::AtomicProposition(const PredicatePtr &predicate)
: predicate_(predicate)
{
}

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::AtomicProposition& s) //NOLINT
    {
        os << *s.predicate();
        return os;
    }
}