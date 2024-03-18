/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/ReasonerModule.h"
#include "knowrob/py/utils.h"
#include "knowrob/URI.h"

using namespace knowrob;
namespace python = boost::python;

ReasonerModule::ReasonerModule(std::string_view modulePath, std::string_view reasonerType)
		: modulePath_(resolveModulePath(modulePath)),
		  reasonerType_(reasonerType) {
}

ReasonerModule::~ReasonerModule()
= default;

std::string ReasonerModule::resolveModulePath(std::string_view modulePath) {
	// TODO: also support modulePath that use "." as delimiter as this is the
	//  common way to address python modules. only std::filesystem::path is used at the moment.
	return URI::resolve(modulePath);
}

bool ReasonerModule::isLoaded() {
	return py::call<bool>([&] {
		return pyReasonerType_ && !pyReasonerType_.is_none();
	});
}

bool ReasonerModule::loadModule() {
	// try to make sure that the module can be imported.
	// the modules can be addressed either via an absolute path,
	// or via a relative path in the KnowRob project.
	std::filesystem::path modulePath(modulePath_);
	if (!std::filesystem::exists(modulePath)) {
		KB_ERROR("Module '{}' does not exist.", modulePath_.c_str());
		return false;
	}

	// For now the directory that contains the python source is simply added to the
	// sys.path of the python interpreter.
	// TODO: this should be done in a more robust way.
	auto moduleDir = modulePath.parent_path();
	auto moduleFile = modulePath.stem();

	try {
		// make sure that the module directory is only added once
		if (moduleDirectories_.count(moduleDir) == 0) {
			moduleDirectories_.insert(moduleDir);
			// >>> sys.path.append(moduleDir)
			auto py_sys = python::import("sys");
			auto py_path = py_sys.attr("path");
			auto sysPathAppend = py_path.attr("append");
			sysPathAppend(moduleDir.string());
		}

		pyModule_ = python::import(moduleFile.c_str());
		if (pyModule_.is_none()) {
			KB_ERROR("Failed to import module '{}'.", modulePath_.c_str());
			return false;
		}
		pyReasonerType_ = pyModule_.attr(reasonerType_.c_str());
	} catch (const boost::python::error_already_set &) {
		throw PythonError();
	}
	return isLoaded();
}

std::shared_ptr<Reasoner> ReasonerModule::createReasoner(std::string_view reasonerID) {
	try {
		// create a reasoner object
		// note: for some reason it does not fly below if the Python reasoner has both Reasoner and
		// DataBackend as base classes. Hence, the introduction of ReasonerWithBackend as a workaround.
		// TODO: plugins also receive reasoner id as argument, include it for modules too.
		//python::object pyReasoner = pyReasonerType_(reasonerID);
		python::object pyReasoner = pyReasonerType_();

		// extract the reasoner in appropriate type
		python::extract<std::shared_ptr<Reasoner>> extracted(pyReasoner);
		if (extracted.check()) {
			return extracted();
		} else {
			KB_ERROR("Failed to extract typed reasoner from module '{}'", modulePath_.c_str());
		}
	} catch (const boost::python::error_already_set &) {
		throw PythonError();
	}
	KB_ERROR("Failed to create reasoner from module '{}'", modulePath_.c_str());
	return {};
}
