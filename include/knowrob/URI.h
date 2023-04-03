//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_URI_H
#define KNOWROB_URI_H

#include <string>

namespace knowrob {
    class URI {
    public:
        static std::string resolve(const std::string &uriString);
    };
}

#endif //KNOWROB_URI_H
