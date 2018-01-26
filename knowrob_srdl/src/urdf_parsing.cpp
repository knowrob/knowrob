#include "rosprolog.h"
#include <string>
#include <vector>

PREDICATE(foo, 1) {
    PL_A1 = std::string("foo").c_str();
    return true;
}

PREDICATE(bar, 1) {
    PL_A1 = std::string("bar").c_str();
    return true;
}

PREDICATE_NONDET(foo_bar, 1) {
//    switch( PL_foreign_control(handle) ) {
//
//    }
    return false;
}
