#include "rosprolog.h"
#include <string>

PREDICATE(foo, 1) {
    PL_A1 = std::string("foo").c_str();
    PL_A1 = std::string("foo2").c_str();
    return true;
}

PREDICATE(bar, 1) {
    PL_A1 = std::string("bar").c_str();
    return true;
}

// PREDICATE(foo_bar, 1) {
//     PlTail l("foo");
//     l.append("bar");
//     PL_A1 = l;
//     return true;
// }
