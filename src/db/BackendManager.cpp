/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>
#include <utility>

#include "knowrob/Logger.h"
#include "knowrob/db/BackendManager.h"
#include "knowrob/db/BackendError.h"

using namespace knowrob;

std::map<uint32_t, BackendManager *> BackendManager::backendManagers_ = {};
uint32_t BackendManager::managerIDCounter_ = 0;

auto &getBackendFactories() {
	static std::map<std::string, std::shared_ptr<BackendFactory>> x;
	return x;
}

BackendManager::BackendManager(KnowledgeBase *kb)
		: kb_(kb), backendIndex_(0) {
	std::lock_guard<std::mutex> scoped_lock(staticMutex_);
	managerID_ = (managerIDCounter_++);
	backendManagers_[managerID_] = this;
}

BackendManager::~BackendManager() {
	std::lock_guard<std::mutex> scoped_lock(staticMutex_);
	backendManagers_.erase(managerID_);
}

BackendManager *BackendManager::getManager(uint32_t managerID) {
	auto it = backendManagers_.find(managerID);
	if (it != backendManagers_.end()) {
		return it->second;
	} else {
		return nullptr;
	}
}

void BackendManager::loadBackend(const boost::property_tree::ptree &config) {
	auto lib = config.get_optional<std::string>("lib");
	auto type = config.get_optional<std::string>("type");
	auto name = config.get_optional<std::string>("name");

	// get a reasoner factory
	std::shared_ptr<BackendFactory> factory;
	if (lib.has_value()) {
		// use factory in DLL
		factory = loadBackendPlugin(lib.value());
	} else if (type.has_value()) {
		auto &backendFactories = getBackendFactories();
		// map type name to a factory
		const auto &it = backendFactories.find(type.value());
		if (it == backendFactories.end()) {
			KB_WARN("no factory registered for backend with type '{}'.", type.value());
		} else {
			factory = it->second;
		}
	} else {
		KB_WARN("missing 'type' or 'lib' key in backend config.");
	}
	// make sure factory was found above
	if (!factory) {
		throw BackendError("failed to load a backend.");
	}
	// create a reasoner id, or use name property
	std::string backendID;
	if (name.has_value()) {
		backendID = name.value();
	} else {
		backendID = factory->name() + std::to_string(backendIndex_);
	}
	KB_INFO("Using backend `{}` with type `{}`.", backendID, factory->name());

	// create a new DataBackend instance
	auto definedBackend = factory->createBackend(backendID);

	ReasonerConfig reasonerConfig(&config);
	if (!definedBackend->backend()->loadConfig(reasonerConfig)) {
		KB_WARN("Backend `{}` failed to loadConfig.", backendID);
	} else {
		addBackend(definedBackend);
	}
	// increase reasonerIndex_
	backendIndex_ += 1;
}

std::shared_ptr<BackendPlugin> BackendManager::loadBackendPlugin(const std::string &path) {
	auto absPath = std::filesystem::absolute(path);
	auto it = loadedPlugins_.find(absPath);
	if (it == loadedPlugins_.end()) {
		auto p = std::make_shared<BackendPlugin>(absPath);
		auto jt = loadedPlugins_.insert(std::pair<std::string,
				std::shared_ptr<BackendPlugin>>(absPath, p));
		if (jt.first->second->loadDLL()) {
			return jt.first->second;
		}
	} else if (it->second->isLoaded()) {
		return it->second;
	}
	KB_WARN("Failed to open backend library at path '{}'.", path);
	return {};
}

bool BackendManager::addFactory(const std::string &typeName, const std::shared_ptr<BackendFactory> &factory) {
	auto &backendFactories = getBackendFactories();
	if (backendFactories.find(typeName) != backendFactories.end()) {
		KB_WARN("overwriting factory for backend type '{}'", typeName);
	}
	backendFactories[typeName] = factory;
	return true;
}

std::shared_ptr<DefinedBackend> BackendManager::addBackend(
		const std::string &backendID, const DataBackendPtr &backend) {
	if (backendPool_.find(backendID) != backendPool_.end()) {
		KB_WARN("overwriting backend with name '{}'", backendID);
	}
	auto managedBackend = std::make_shared<DefinedBackend>(backendID, backend);
	backendPool_[backendID] = managedBackend;

	return managedBackend;
}

void BackendManager::addBackend(const std::shared_ptr<DefinedBackend> &definedKG) {
	if (backendPool_.find(definedKG->name()) != backendPool_.end()) {
		KB_WARN("overwriting backend with name '{}'", definedKG->name());
	}
	backendPool_[definedKG->name()] = definedKG;
}

void BackendManager::removeBackend(const std::shared_ptr<DefinedBackend> &backend) {
	backendPool_.erase(backend->name());
}

std::shared_ptr<DefinedBackend> BackendManager::getBackendWithID(const std::string &backendID) {
	auto it = backendPool_.find(backendID);
	if (it != backendPool_.end()) {
		return it->second;
	} else {
		return {};
	}
}
