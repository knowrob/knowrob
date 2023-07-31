//
// Created by daniel on 31.07.23.
//

#include "knowrob/semweb/PrefixProbe.h"

namespace knowrob {
    bool operator<(PrefixProbe a, std::string_view b) { return a.prefix < b.substr(0, a.prefix.size()); }
    bool operator<(std::string_view a, PrefixProbe b) { return a.substr(0, b.prefix.size()) < b.prefix; }
} // knowrob
