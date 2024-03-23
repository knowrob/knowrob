/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>

#include "knowrob/Logger.h"
#include "knowrob/backend/BackendManager.h"
#include "knowrob/backend/BackendError.h"
#include "knowrob/backend/QueryableBackend.h"

using namespace knowrob;

BackendManager::BackendManager(const std::shared_ptr<Vocabulary> &vocabulary)
		: PluginManager(),
		  vocabulary_(vocabulary) {
}

std::shared_ptr<NamedBackend> BackendManager::loadPlugin(const boost::property_tree::ptree &config) {
	// get a backend factory
	std::shared_ptr<BackendFactory> factory = findFactory(config);
	// make sure factory was found above
	if (!factory) throw BackendError("failed to load a backend.");
	// create a reasoner id, or use name property
	std::string backendID = getPluginID(factory, config);
	KB_INFO("Using backend `{}` with type `{}`.", backendID, factory->name());

	// create a new DataBackend instance
	auto definedBackend = factory->create(backendID);
	definedBackend->value()->setVocabulary(vocabulary());

	PropertyTree pluginConfig(&config);
	if (!definedBackend->value()->initializeBackend(pluginConfig)) {
		KB_WARN("Backend `{}` failed to loadConfig.", backendID);
	} else {
		addPlugin(definedBackend);
	}

	return definedBackend;
}

std::shared_ptr<NamedBackend> BackendManager::addPlugin(std::string_view backendID, const DataBackendPtr &backend) {
	if (pluginPool_.find(backendID) != pluginPool_.end()) {
		KB_WARN("overwriting backend with name '{}'", backendID);
	}
	auto managedBackend = std::make_shared<NamedBackend>(backendID, backend);
	pluginPool_.emplace(backendID, managedBackend);
	initBackend(managedBackend);
	return managedBackend;
}

void BackendManager::addPlugin(const std::shared_ptr<NamedBackend> &definedKG) {
	if (pluginPool_.find(definedKG->name()) != pluginPool_.end()) {
		KB_WARN("overwriting backend with name '{}'", definedKG->name());
	}
	pluginPool_[definedKG->name()] = definedKG;
	initBackend(definedKG);
}

void BackendManager::initBackend(const std::shared_ptr<NamedBackend> &definedKG) {
	definedKG->value()->setVocabulary(vocabulary());
	// check if the backend is a QueryableBackend, if so store it in the queryable_ map
	auto queryable = std::dynamic_pointer_cast<QueryableBackend>(definedKG->value());
	if (queryable) {
		KB_INFO("adding queryable backend with id '{}'.", definedKG->name());
		queryable_[definedKG->name()] = queryable;
	}
	// check if the backend is a PersistentBackend, if so store it in the persistent_ map
	if (queryable && queryable->isPersistent()) {
		KB_INFO("adding persistent backend with id '{}'.", definedKG->name());
		persistent_[definedKG->name()] = queryable;
	}
}
