/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <dlfcn.h>
#include "knowrob/backend/BackendPlugin.h"

using namespace knowrob;

BackendPlugin::BackendPlugin(std::string dllPath)
		: handle_(nullptr),
		  create_(nullptr),
		  get_name_(nullptr),
		  dllPath_(std::move(dllPath)) {
}

BackendPlugin::~BackendPlugin() {
	if (handle_) {
		dlclose(handle_);
		handle_ = nullptr;
	}
}

bool BackendPlugin::isLoaded() {
	return (create_ != nullptr && get_name_ != nullptr);
}

bool BackendPlugin::loadDLL() {
	handle_ = dlopen(dllPath_.c_str(), RTLD_LAZY);
	if (handle_ != nullptr) {
		create_ = (std::shared_ptr<DataBackend> (*)())
				dlsym(handle_, "knowrob_createBackend");
		get_name_ = (char *(*)())
				dlsym(handle_, "knowrob_getPluginName");
		return isLoaded();
	} else {
		return false;
	}
}

std::shared_ptr<DefinedBackend> BackendPlugin::createBackend(const std::string &reasonerID) {
	return std::make_shared<DefinedBackend>(reasonerID, create_());
}
