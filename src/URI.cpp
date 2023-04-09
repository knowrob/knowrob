//
// Created by daniel on 01.04.23.
//

#include <filesystem>
#include "knowrob/URI.h"

using namespace knowrob;

std::string URI::resolve(const std::string_view &uriString)
{
    static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
    static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

    std::filesystem::path filePath(uriString);
    if(!exists(filePath)) {
        auto possiblePaths = {
                projectPath / filePath,
                projectPath / "src" / filePath,
                installPath / "share" / "knowrob" / filePath
        };
        for(const auto &p : possiblePaths) {
            if(exists(p)) return p.u8string();
        }
    }
    return std::string(uriString);
}
