/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/ReasonerManager.h>
#include <knowrob/prolog/PrologReasoner.h>
#include <knowrob/mongolog/MongologReasoner.h>
// shared libraries
#include <dlfcn.h>

using namespace knowrob;


ReasonerManager::ReasonerManager()
: reasonerIndex_(0)
{
	// add some default factory functions to create reasoner instances
	addReasonerFactory("Mongolog", std::make_shared<TypedReasonerFactory<MongologReasoner>>("Mongolog"));
	addReasonerFactory("Prolog",   std::make_shared<TypedReasonerFactory<PrologReasoner>>("Prolog"));
}

void ReasonerManager::loadReasoner(const boost::property_tree::ptree &config)
{
	// get a reasoner factory
	std::shared_ptr<ReasonerFactory> factory;
	if(config.count("lib")) {
		// use factory in DLL
		const auto &libPath = config.get<std::string>("lib");
		factory = loadReasonerPlugin(libPath);
	}
	else if(config.count("type")) {
		// map type name to a factory
		const auto &typeName = config.get<std::string>("type");
		const auto &it = reasonerFactories_.find(typeName);
		if(it == reasonerFactories_.end()) {
			KB_WARN("no factory registered for reasoner type '{}'.", typeName);
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
	if(config.count("name")) {
		reasonerID = config.get<std::string>("name");
	}
	else {
		reasonerID = factory->name() + std::to_string(reasonerIndex_);
	}
	KB_INFO("Using reasoner `{}` with type `{}`.", reasonerID, factory->name());
	// create a new reasoner instance
	auto reasoner = factory->createReasoner(reasonerID);
	if(!reasoner->initialize(ReasonerConfiguration(config))) {
		KB_WARN("Reasoner `{}` failed to initialize.", reasonerID);
	}
	else {
		addReasoner(reasoner);
	}
	// increase reasonerIndex_
	reasonerIndex_ += 1;
}

std::shared_ptr<ReasonerPlugin> ReasonerManager::loadReasonerPlugin(const std::string &path)
{
	// TODO: map path to absolute path to avoid loading DLLs multiple times.
	auto it = loadedPlugins_.find(path);
	if(it == loadedPlugins_.end()) {
		auto p = std::make_shared<ReasonerPlugin>(path);
		auto jt = loadedPlugins_.insert(std::pair<std::string, std::shared_ptr<ReasonerPlugin>>(path, p));
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

void ReasonerManager::addReasonerFactory(const std::string &typeName, const std::shared_ptr<ReasonerFactory> &factory)
{
	reasonerFactories_[typeName] = factory;
}

void ReasonerManager::addReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	reasonerPool_.push_back(reasoner);
}

void ReasonerManager::removeReasoner(const std::shared_ptr<IReasoner> &reasoner)
{
	reasonerPool_.remove(reasoner);
}

std::list<std::shared_ptr<IReasoner>> ReasonerManager::getReasonerForPredicate(const PredicateIndicator &predicate)
{
	std::list<std::shared_ptr<IReasoner>> out;
	for(auto &x : reasonerPool_) {
		if(x->isCurrentPredicate(predicate)) {
			out.push_back(x);
		}
	}
	return out;
}

/******************************************/
/************ ReasonerPlugin **************/
/******************************************/

ReasonerPlugin::ReasonerPlugin(const std::string &dllPath)
		: handle_(nullptr),
		  create_(nullptr),
		  get_name_(nullptr),
		  dllPath_(dllPath)
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

ReasonerConfiguration::ReasonerConfiguration(const boost::property_tree::ptree &config)
{
	for(const auto &pair : config.get_child("data-sources")) {
		auto &subtree = pair.second;

		if(subtree.count("file")) {
			const auto &fileName = subtree.get<std::string>("file");
			dataFiles.push_back(std::make_shared<DataFile>(fileName));
		}
		else {
			KB_WARN("cannot load data source");
		}
	}
}
