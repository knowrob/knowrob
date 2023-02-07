/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <filesystem>
#include <utility>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/reasoner.h>
// loading shared libraries
#include <dlfcn.h>

using namespace knowrob;

/******************************************/
/*************** IReasoner ****************/
/******************************************/

void IReasoner::addDataFileHandler(const std::string &format, const DataFileLoader &fn)
{
	dataFileHandler_[format] = fn;
}

bool IReasoner::loadDataFile(const DataFilePtr &dataFile)
{
	if(dataFile->hasUnknownFormat()) {
		return loadDataFileWithUnknownFormat(dataFile);
	}
	else {
		auto it = dataFileHandler_.find(dataFile->format());
		if(it == dataFileHandler_.end()) {
			KB_WARN("Ignoring data file with unknown format \"{}\"", dataFile->format());
			return false;
		}
		else {
			KB_INFO("Using data file {} with format \"{}\".", dataFile->path(), dataFile->format());
			return it->second(dataFile);
		}
	}
}

/******************************************/
/************ ReasonerManager *************/
/******************************************/

std::map<std::string, std::shared_ptr<ReasonerFactory>> ReasonerManager::reasonerFactories_ = {};

ReasonerManager::ReasonerManager()
: reasonerIndex_(0)
{
}

void ReasonerManager::loadReasoner(const boost::property_tree::ptree &config)
{
	auto lib = config.get_optional<std::string>("lib");
	auto type = config.get_optional<std::string>("type");
	auto name = config.get_optional<std::string>("name");

	// get a reasoner factory
	std::shared_ptr<ReasonerFactory> factory;
	if(lib.has_value()) {
		// use factory in DLL
		factory = loadReasonerPlugin(lib.value());
	}
	else if(type.has_value()) {
		// map type name to a factory
		const auto &it = reasonerFactories_.find(type.value());
		if(it == reasonerFactories_.end()) {
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
		throw ReasonerError("failed to load a reasoner.");
	}
	// create a reasoner id, or use name property
	std::string reasonerID;
	if(name.has_value()) {
		reasonerID = name.value();
	}
	else {
		reasonerID = factory->name() + std::to_string(reasonerIndex_);
	}
	KB_INFO("Using reasoner `{}` with type `{}`.", reasonerID, factory->name());
	// create a new reasoner instance
	auto reasoner = factory->createReasoner(reasonerID);
	ReasonerConfiguration reasonerConfig;
	reasonerConfig.loadPropertyTree(config);
	if(!reasoner->loadConfiguration(reasonerConfig)) {
		KB_WARN("Reasoner `{}` failed to loadConfiguration.", reasonerID);
	}
	else {
		addReasoner(reasonerID, reasoner);
	}
	// increase reasonerIndex_
	reasonerIndex_ += 1;
}

std::shared_ptr<ReasonerPlugin> ReasonerManager::loadReasonerPlugin(const std::string &path)
{
	auto absPath = std::filesystem::absolute(path);
	auto it = loadedPlugins_.find(absPath);
	if(it == loadedPlugins_.end()) {
		auto p = std::make_shared<ReasonerPlugin>(absPath);
		auto jt = loadedPlugins_.insert(std::pair<std::string,
										std::shared_ptr<ReasonerPlugin>>(absPath, p));
		if(jt.first->second->loadDLL()) {
			return jt.first->second;
		}
	}
	else if(it->second->isLoaded()) {
		return it->second;
	}
	KB_WARN("Failed to open reasoner library at path '{}'.", path);
	return {};
}

bool ReasonerManager::addReasonerFactory(const std::string &typeName, const std::shared_ptr<ReasonerFactory> &factory)
{
	if(reasonerFactories_.find(typeName) != reasonerFactories_.end()) {
		KB_WARN("overwriting factory for reasoner type '{}'", typeName);
	}
	reasonerFactories_[typeName] = factory;
	return true;
}

std::shared_ptr<ManagedReasoner> ReasonerManager::addReasoner(const std::string &reasonerID, const std::shared_ptr<IReasoner> &reasoner)
{
	if(reasonerPool_.find(reasonerID) != reasonerPool_.end()) {
		KB_WARN("overwriting reasoner with name '{}'", reasonerID);
	}
	auto managedReasoner = std::make_shared<ManagedReasoner>(reasonerID,reasoner);
	reasonerPool_[reasonerID] = managedReasoner;
	return managedReasoner;
}

void ReasonerManager::removeReasoner(const std::shared_ptr<ManagedReasoner> &reasoner)
{
	reasonerPool_.erase(reasoner->name());
}

std::shared_ptr<ManagedReasoner> ReasonerManager::getReasonerWithID(const std::string &reasonerID)
{
	auto it = reasonerPool_.find(reasonerID);
	if(it != reasonerPool_.end()) {
		return it->second;
	}
	else {
		return {};
	}
}

std::shared_ptr<PredicateDefinition> ReasonerManager::getPredicateDefinition(
		const std::shared_ptr<PredicateIndicator> &indicator)
{
	auto description = std::make_shared<PredicateDefinition>(indicator);
	for(auto &x : reasonerPool_) {
		auto description_n = x.second->reasoner()->getPredicateDescription(indicator);
		if(description_n && !description->addReasoner(x.second, description_n)) {
			KB_WARN("ignoring inconsistent reasoner descriptions provided.");
		}
	}
	return description;
}

PredicateDefinition::PredicateDefinition(const std::shared_ptr<PredicateIndicator> &indicator)
: indicator_(indicator),
  predicateType_(PredicateType::BUILT_IN)
{
}

bool PredicateDefinition::addReasoner(
		const std::shared_ptr<ManagedReasoner> &managedReasoner,
		const std::shared_ptr<PredicateDescription> &description)
{
	if(reasonerEnsemble_.empty()) {
		predicateType_ = description->type();
	}
	else if(predicateType_ != description->type()) {
		// another reasoner has this predicate defined with another type!
		return false;
	}
	reasonerEnsemble_.insert(managedReasoner);
	return true;
}

/******************************************/
/************ ReasonerPlugin **************/
/******************************************/

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
		create_ = (std::shared_ptr<IReasoner> (*)(const std::string&))
				dlsym(handle_, "knowrob_createReasoner");
		get_name_ = (char* (*)())
				dlsym(handle_, "knowrob_getPluginName");
		return isLoaded();
	}
	else {
		return false;
	}
}

std::shared_ptr<IReasoner> ReasonerPlugin::createReasoner(const std::string &reasonerID)
{
	return create_(reasonerID);
}

/******************************************/
/********* ReasonerConfiguration **********/
/******************************************/

void ReasonerConfiguration::loadPropertyTree(const boost::property_tree::ptree &config)
{
	static const std::string formatDefault = {};

	// load all key-value pairs into settings list
	for(const auto& key_val : config) {
		auto key_t = std::make_shared<StringTerm>(key_val.first);
		if(key_val.second.empty()) {
			settings.emplace_back(key_t, std::make_shared<StringTerm>(key_val.second.get_value<std::string>()));
		}
		else {
			loadSettings(key_t, key_val.second);
		}
	}

	auto data_sources = config.get_child_optional("data-sources");
	if(data_sources) {
		for(const auto &pair : data_sources.value()) {
			auto &subtree = pair.second;

			auto fileValue = subtree.get_optional<std::string>("file");
			if(fileValue.has_value()) {
				auto fileFormat = subtree.get("format",formatDefault);
				dataFiles.push_back(std::make_shared<DataFile>(fileValue.value(), fileFormat));
			}
			else {
				KB_WARN("Ignoring data source without \"file\" key.");
			}
		}
	}
}

void ReasonerConfiguration::loadSettings( //NOLINT
		const TermPtr &key1, const boost::property_tree::ptree &ptree)
{
	static auto colon_f = std::make_shared<PredicateIndicator>(":", 2);

	for(const auto& key_val : ptree) {
		if(key_val.first.empty()) {
			// this indicates a list
			// TODO: handle list values here, needs an interface to translate a subtree to a term.
			//		recursive call of loadSettings wouldn't do the trick.
			/*
			auto &listData = key_val.second;
			std::vector<TermPtr> elements(listData.size());
			int index = 0;
			for(const auto& l_key_val : listData) {
				elements[index++] = std::make_shared<StringTerm>(l_key_val.second.get_value<std::string>());
			}
			settings.emplace_back(key1, std::make_shared<ListTerm>(elements));
			*/
			continue;
		}
		else {
			auto key2 = std::make_shared<StringTerm>(key_val.first);
			auto key_t = std::make_shared<Predicate>(Predicate(colon_f, { key1, key2 }));

			if(key_val.second.empty()) {
				settings.emplace_back(key_t, std::make_shared<StringTerm>(key_val.second.get_value<std::string>()));
			}
			else {
				loadSettings(key_t, key_val.second);
			}
		}
	}
}
