/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <set>
#include "knowrob/py/utils.h"
#include "knowrob/URI.h"
#include "knowrob/Logger.h"

namespace knowrob::py {
	std::string resolveModulePath(std::string_view modulePath) {
		// guess if '/' or '.' is used as delimiter
		if (modulePath.find('/') != std::string::npos) {
			return URI::resolve(modulePath);
		} else {
			// replace '.' with '/', assuming dots do not appear in directory names.
			std::string modulePath_withSlash(modulePath);
			std::replace(modulePath_withSlash.begin(), modulePath_withSlash.end(), '.', '/');
			return URI::resolve(modulePath_withSlash);
		}
	}

	std::string addToSysPath(const std::filesystem::path& modulePath) {
		static std::set<std::filesystem::path> moduleDirectories;

		auto topmostPythonPath = modulePath.parent_path();
		std::string relativeModulePath = modulePath.stem().string();
		while(std::filesystem::exists(topmostPythonPath / "__init__.py")) {
			topmostPythonPath = topmostPythonPath.parent_path();
			relativeModulePath.insert(0, ".");
			relativeModulePath.insert(0, topmostPythonPath.stem().string());
		}

		// make sure that the module directory is only added once
		if (moduleDirectories.count(topmostPythonPath) == 0) {
			moduleDirectories.insert(topmostPythonPath);
			// >>> sys.path.append(moduleDir)
			auto py_sys = boost::python::import("sys");
			auto py_path = py_sys.attr("path");
			auto sysPathAppend = py_path.attr("append");
			sysPathAppend(topmostPythonPath.string());
			KB_DEBUG("[python] Added '{}' to sys.path.", topmostPythonPath.string().c_str());
		}

		return relativeModulePath;
	}
}
