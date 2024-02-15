//
// Created by daniel on 01.04.23.
//

#include <filesystem>
#include "knowrob/URI.h"

using namespace knowrob;

URI::URI(std::string_view path)
		: path_(path) {
	updateURI();
}

URI::URI(std::string path, std::string protocol, std::string host, int port)
		: protocol_(std::move(protocol)),
		  path_(std::move(path)),
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

std::string URI::resolve(const std::string_view &uriString) {
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	std::filesystem::path filePath(uriString);
	if (!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				projectPath / "src" / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for (const auto &p: possiblePaths) {
			if (exists(p)) return p.u8string();
		}
	}
	return std::string(uriString);
}
