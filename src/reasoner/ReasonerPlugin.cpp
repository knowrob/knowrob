/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <dlfcn.h>
#include "knowrob/reasoner/ReasonerPlugin.h"

using namespace knowrob;

ReasonerPlugin::ReasonerPlugin(std::string_view dllPath)
		: handle_(nullptr),
		  create_(nullptr),
		  get_name_(nullptr),
		  dllPath_(dllPath) {
}

ReasonerPlugin::~ReasonerPlugin() {
	if (handle_) {
		dlclose(handle_);
		handle_ = nullptr;
	}
}

bool ReasonerPlugin::isLoaded() {
	return (create_ != nullptr && get_name_ != nullptr);
}

bool ReasonerPlugin::loadDLL() {
	handle_ = dlopen(dllPath_.c_str(), RTLD_LAZY);
	if (handle_ != nullptr) {
		create_ = (std::shared_ptr<Reasoner> (*)(std::string_view))
				dlsym(handle_, "knowrob_createReasoner");
		get_name_ = (char *(*)())
				dlsym(handle_, "knowrob_getPluginName");
		return isLoaded();
	} else {
		return false;
	}
}

std::shared_ptr<Reasoner> ReasonerPlugin::createReasoner(std::string_view reasonerID) {
	return create_(reasonerID);
}
