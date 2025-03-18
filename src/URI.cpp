/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef _WIN32

#include <pwd.h>

#endif

#include <filesystem>
#include "knowrob/URI.h"
#include "knowrob/Logger.h"
#include <optional>

using namespace knowrob;

URI::URI(std::string_view path)
		: path_(path) {
	updateURI();
}

URI::URI(std::string path, std::string protocol, std::string host, int port)
		: path_(std::move(path)),
		  protocol_(std::move(protocol)),
		  host_(std::move(host)),
		  port_(port) {
	updateURI();
}

URI::URI(const boost::property_tree::ptree &property_tree) {
	protocol_ = property_tree.get_optional<std::string>("protocol");
	path_ = property_tree.get("path", "/");
	host_ = property_tree.get_optional<std::string>("host");
	port_ = property_tree.get_optional<int>("port");
	updateURI();
}

void URI::updateURI() {
	std::stringstream ss;
	if (protocol_ && protocol_.value() != "file") {
		ss << protocol_ << "://";
	}
	if (host_) {
		ss << host_.value();
		if (port_) ss << ':' << port_.value();
	}
	ss << path_;
	uri_ = ss.str();
}

std::optional<std::string> URI::getHomePath(void) {
#ifdef _WIN32
	auto env_drive = getenv("HOMEDRIVE");
	auto env_path = getenv("HOMEPATH");
	if (env_drive && env_path) {
			return std::string(env_drive) + std::string(env_path);
	} else {
		KB_WARN("Could not determine home directory");
		return std::nullopt;
	}
#else
	auto env_home = getenv("HOME");
	if (env_home) {
		return std::string(env_home);
	} else {
		struct passwd *pw = getpwuid(getuid());
		if (pw) {
			return std::string(pw->pw_dir);
		} else {
			KB_WARN("Could not determine home directory");
			return std::nullopt;
		}
	}
#endif
}

std::string URI::resolve(const std::string_view &uriString) {
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);
	static std::filesystem::path buildPath(KNOWROB_BUILD_DIR);
	static std::optional<std::string> homePath(getHomePath());

	std::filesystem::path filePath(uriString);
	if (!exists(filePath)) {
		std::vector<std::filesystem::path> possiblePaths;
		// prefer loading from source directory
		possiblePaths.push_back(projectPath / filePath);
		possiblePaths.push_back(projectPath / "src" / filePath);
		// next try loading from home directory
		if (homePath) {
			possiblePaths.push_back(std::filesystem::path(homePath.value()) / ".knowrob" / filePath);
		}
		// try loading from install directory
		possiblePaths.push_back(installPath / "share" / "knowrob" / filePath);
		possiblePaths.push_back(installPath / "lib" / "knowrob" / filePath);
		// lastly try loading from build directory
		possiblePaths.push_back(buildPath / filePath);

		for (const auto &p: possiblePaths) {
			if (exists(p)) return p.u8string();
		}
	}
	return std::string(uriString);
}
