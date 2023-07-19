/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>
#include <utility>

#include "knowrob/Logger.h"
#include "knowrob/backend/KnowledgeGraphManager.h"
#include "knowrob/backend/KnowledgeGraphError.h"

using namespace knowrob;

std::map<std::string, std::shared_ptr<KnowledgeGraphFactory>> KnowledgeGraphManager::backendFactories_ = {};
std::map<uint32_t, KnowledgeGraphManager*> KnowledgeGraphManager::backendManagers_ = {};
uint32_t KnowledgeGraphManager::managerIDCounter_ = 0;

KnowledgeGraphManager::KnowledgeGraphManager(const std::shared_ptr<ThreadPool> &threadPool)
: threadPool_(threadPool), backendIndex_(0)
{
    std::lock_guard<std::mutex> scoped_lock(staticMutex_);
    managerID_ = (managerIDCounter_++);
    backendManagers_[managerID_] = this;
}

KnowledgeGraphManager::~KnowledgeGraphManager()
{
    std::lock_guard<std::mutex> scoped_lock(staticMutex_);
    backendManagers_.erase(managerID_);
}

KnowledgeGraphManager* KnowledgeGraphManager::getManager(uint32_t managerID)
{
    auto it = backendManagers_.find(managerID);
    if(it != backendManagers_.end()) {
        return it->second;
    } else {
        return nullptr;
    }
}

void KnowledgeGraphManager::loadKnowledgeGraph(const boost::property_tree::ptree &config)
{
	auto lib = config.get_optional<std::string>("lib");
	auto type = config.get_optional<std::string>("type");
	auto name = config.get_optional<std::string>("name");

	// get a reasoner factory
	std::shared_ptr<KnowledgeGraphFactory> factory;
	if(lib.has_value()) {
		// use factory in DLL
		factory = loadBackendPlugin(lib.value());
	}
	else if(type.has_value()) {
		// map type name to a factory
		const auto &it = backendFactories_.find(type.value());
		if(it == backendFactories_.end()) {
			KB_WARN("no factory registered for reasoner type '{}'.", type.value());
		}
		else {
			factory = it->second;
		}
	}
	else {
		KB_WARN("missing 'type' or 'lib' key in reasoner config.");
	}
	// make sure factory was found above
	if(!factory) {
		throw KnowledgeGraphError("failed to load a reasoner.");
	}
	// create a reasoner id, or use name property
	std::string backendID;
	if(name.has_value()) {
		backendID = name.value();
	}
	else {
		backendID = factory->name() + std::to_string(backendIndex_);
	}
	KB_INFO("Using knowledgeGraph `{}` with type `{}`.", backendID, factory->name());

	// create a new reasoner instance
	auto backend = factory->createKnowledgeGraph(backendID);
	backend->setThreadPool(threadPool_);

	if(!backend->loadConfiguration(config)) {
		KB_WARN("Backend `{}` failed to loadConfiguration.", backendID);
	}
	else {
        addKnowledgeGraph(backendID, backend);
	}
	// increase reasonerIndex_
	backendIndex_ += 1;
}

std::shared_ptr<KnowledgeGraphPlugin> KnowledgeGraphManager::loadBackendPlugin(const std::string &path)
{
	auto absPath = std::filesystem::absolute(path);
	auto it = loadedPlugins_.find(absPath);
	if(it == loadedPlugins_.end()) {
		auto p = std::make_shared<KnowledgeGraphPlugin>(absPath);
		auto jt = loadedPlugins_.insert(std::pair<std::string,
										std::shared_ptr<KnowledgeGraphPlugin>>(absPath, p));
		if(jt.first->second->loadDLL()) {
			return jt.first->second;
		}
	}
	else if(it->second->isLoaded()) {
		return it->second;
	}
	KB_WARN("Failed to open knowledgeGraph library at path '{}'.", path);
	return {};
}

bool KnowledgeGraphManager::addFactory(const std::string &typeName, const std::shared_ptr<KnowledgeGraphFactory> &factory)
{
	if(backendFactories_.find(typeName) != backendFactories_.end()) {
		KB_WARN("overwriting factory for knowledgeGraph type '{}'", typeName);
	}
	backendFactories_[typeName] = factory;
	return true;
}

std::shared_ptr<DefinedKnowledgeGraph> KnowledgeGraphManager::addKnowledgeGraph(
        const std::string &backendID, const std::shared_ptr<KnowledgeGraph> &kg)
{
	if(knowledgeGraphPool_.find(backendID) != knowledgeGraphPool_.end()) {
		KB_WARN("overwriting knowledgeGraph with name '{}'", backendID);
	}
	auto managedBackend = std::make_shared<DefinedKnowledgeGraph>(backendID, kg);
    knowledgeGraphPool_[backendID] = managedBackend;

	return managedBackend;
}

void KnowledgeGraphManager::removeBackend(const std::shared_ptr<DefinedKnowledgeGraph> &backend)
{
	knowledgeGraphPool_.erase(backend->name());
}

std::shared_ptr<DefinedKnowledgeGraph> KnowledgeGraphManager::getKnowledgeGraphWithID(const std::string &backendID)
{
	auto it = knowledgeGraphPool_.find(backendID);
	if(it != knowledgeGraphPool_.end()) {
		return it->second;
	}
	else {
		return {};
	}
}
