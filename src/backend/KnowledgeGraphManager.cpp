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

std::map<uint32_t, KnowledgeGraphManager*> KnowledgeGraphManager::backendManagers_ = {};
uint32_t KnowledgeGraphManager::managerIDCounter_ = 0;

auto& getBackendFactories() {
    static std::map<std::string, std::shared_ptr<KnowledgeGraphFactory>> x;
    return x;
}

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
	    auto &backendFactories = getBackendFactories();
		// map type name to a factory
		const auto &it = backendFactories.find(type.value());
		if(it == backendFactories.end()) {
			KB_WARN("no factory registered for knowledge graph with type '{}'.", type.value());
		}
		else {
			factory = it->second;
		}
	}
	else {
		KB_WARN("missing 'type' or 'lib' key in knowledge graph config.");
	}
	// make sure factory was found above
	if(!factory) {
		throw KnowledgeGraphError("failed to load a knowledge graph.");
	}
	// create a reasoner id, or use name property
	std::string backendID;
	if(name.has_value()) {
		backendID = name.value();
	}
	else {
		backendID = factory->name() + std::to_string(backendIndex_);
	}
	KB_INFO("Using knowledge graph `{}` with type `{}`.", backendID, factory->name());

	// create a new reasoner instance
	auto backend = factory->createKnowledgeGraph(backendID);
	backend->knowledgeGraph()->setThreadPool(threadPool_);

	if(!backend->knowledgeGraph()->loadConfiguration(config)) {
		KB_WARN("Backend `{}` failed to loadConfiguration.", backendID);
	}
	else {
        addKnowledgeGraph(backend);
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
	KB_WARN("Failed to open knowledge graph library at path '{}'.", path);
	return {};
}

bool KnowledgeGraphManager::addFactory(const std::string &typeName, const std::shared_ptr<KnowledgeGraphFactory> &factory)
{
    auto &backendFactories = getBackendFactories();
	if(backendFactories.find(typeName) != backendFactories.end()) {
		KB_WARN("overwriting factory for knowledge graph type '{}'", typeName);
	}
	backendFactories[typeName] = factory;
	return true;
}

std::shared_ptr<DefinedKnowledgeGraph> KnowledgeGraphManager::addKnowledgeGraph(
        const std::string &backendID, const std::shared_ptr<KnowledgeGraph> &kg)
{
	if(knowledgeGraphPool_.find(backendID) != knowledgeGraphPool_.end()) {
		KB_WARN("overwriting knowledge graph with name '{}'", backendID);
	}
	auto managedBackend = std::make_shared<DefinedKnowledgeGraph>(backendID, kg);
    knowledgeGraphPool_[backendID] = managedBackend;

	return managedBackend;
}

void KnowledgeGraphManager::addKnowledgeGraph(const std::shared_ptr<DefinedKnowledgeGraph> &definedKG)
{
	if(knowledgeGraphPool_.find(definedKG->name()) != knowledgeGraphPool_.end()) {
		KB_WARN("overwriting knowledge graph with name '{}'", definedKG->name());
	}
    knowledgeGraphPool_[definedKG->name()] = definedKG;
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

std::shared_ptr<semweb::Vocabulary> KnowledgeGraphManager::vocabulary()
{
    // TODO: if each KG has the same Vocabulary, it should be managed in KnowledgeGraphManager instead!
    if(knowledgeGraphPool_.empty()) {
        return {};
    }
    else {
        return knowledgeGraphPool_.begin()->second->knowledgeGraph()->vocabulary();
    }
}

std::shared_ptr<semweb::ImportHierarchy> KnowledgeGraphManager::importHierarchy()
{
    // TODO: if each KG has the same ImportHierarchy, it should be managed in KnowledgeGraphManager instead!
    if(knowledgeGraphPool_.empty()) {
        return {};
    }
    else {
        return knowledgeGraphPool_.begin()->second->knowledgeGraph()->importHierarchy();
    }
}
