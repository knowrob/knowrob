//
// Created by daniel on 31.07.23.
//

#ifndef KNOWROB_PREFIX_PROBE_H
#define KNOWROB_PREFIX_PROBE_H

#include "string"

namespace knowrob {
    struct PrefixProbe { std::string_view prefix; };
    bool operator<(PrefixProbe a, std::string_view b);
    bool operator<(std::string_view a, PrefixProbe b);
} // knowrob

#endif //KNOWROB_PREFIX_PROBE_H
