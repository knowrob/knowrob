/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <dlfcn.h>
#include "knowrob/reasoner/ReasonerPlugin.h"

using namespace knowrob;

ReasonerPlugin::ReasonerPlugin(std::string dllPath)
		: handle_(nullptr),
		  create_(nullptr),
		  get_name_(nullptr),
		  dllPath_(std::move(dllPath))
{
}

ReasonerPlugin::~ReasonerPlugin()
{
	if(handle_) {
		dlclose(handle_);
		handle_ = nullptr;
	}
}

bool ReasonerPlugin::isLoaded()
{
	return (create_ != nullptr && get_name_ != nullptr);
}

bool ReasonerPlugin::loadDLL()
{
	handle_ = dlopen(dllPath_.c_str(), RTLD_LAZY);
	if(handle_ != nullptr) {
		create_ = (std::shared_ptr<Reasoner> (*)(const std::string&))
				dlsym(handle_, "knowrob_createReasoner");
		get_name_ = (char* (*)())
				dlsym(handle_, "knowrob_getPluginName");
		return isLoaded();
	}
	else {
		return false;
	}
}

std::shared_ptr<Reasoner> ReasonerPlugin::createReasoner(const std::string &reasonerID)
{
	return create_(reasonerID);
}
