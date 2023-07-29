/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KG_MANAGER_H_
#define KNOWROB_KG_MANAGER_H_

#include <map>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include "TypedKnowledgeGraphFactory.h"
#include "KnowledgeGraphPlugin.h"
#include "DefinedKnowledgeGraph.h"
#include "KnowledgeGraph.h"

namespace knowrob {
	/**
	 * Manages a set of available KG subsystems.
	 */
	class KnowledgeGraphManager {
	public:
		explicit KnowledgeGraphManager(const std::shared_ptr<ThreadPool> &threadPool);
        ~KnowledgeGraphManager();

        /**
         * @param managerID the ID of a KG manager
         * @return the KG manager, or nullptr if ID is unknown
         */
        static KnowledgeGraphManager* getManager(uint32_t managerID);

		/**
		 * Add a KG factory to the manager.
		 * Note that factories for shared libraries are created on the fly, and thus
		 * do not need to be added manually.
		 * @param typeName the name of the KG type
		 * @param factory a KG factory
		 */
		static bool addFactory(const std::string &typeName, const std::shared_ptr<KnowledgeGraphFactory> &factory);

		template<class T> static bool addFactory(const std::string &typeName)
		{ return addFactory(typeName, std::make_shared<TypedKnowledgeGraphFactory<T>>(typeName)); }

		/**
		 * Load a new backend instance into the KG manager.
		 * The type of the KG is determined based on either the value of
		 * "type" or "lib" in the property tree root. The tree is further used
		 * to generate a backend configuration used by the created KG.
		 * Backend factories for libraries are created on the fly, the ones
		 * for built-in backend types need to be added to the KG manager before.
		 * @param config a property tree holding a KG configuration
		 */
		void loadKnowledgeGraph(const boost::property_tree::ptree &config);

		/**
		 * @param backendID a KG ID string.
		 * @return a KG instance or a null pointer reference.
		 */
		std::shared_ptr<DefinedKnowledgeGraph> getKnowledgeGraphWithID(const std::string &backendID);

        /**
         * Add a KG to this manager.
         * @reasoner a defined KG.
         */
        std::shared_ptr<DefinedKnowledgeGraph> addKnowledgeGraph(
                const std::string &reasonerID, const std::shared_ptr<KnowledgeGraph> &backend);

        void addKnowledgeGraph(const std::shared_ptr<DefinedKnowledgeGraph> &backend);

        /**
         * @return map of all KG defined by this manager.
         */
        const auto& knowledgeGraphPool() const  { return knowledgeGraphPool_; }

        auto managerID() const  { return managerID_; }

	private:
        std::shared_ptr<ThreadPool> threadPool_;
        // maps backend id to manager
        static std::map<uint32_t, KnowledgeGraphManager*> backendManagers_;
        // counts number of initialized managers
        static uint32_t managerIDCounter_;
        // mutex used to interact with static variables
        std::mutex staticMutex_;
		// pool of all backend instances created via this manager
		// maps backend ID to backend instance.
		std::map<std::string, std::shared_ptr<DefinedKnowledgeGraph>> knowledgeGraphPool_;
		// maps plugin names to factories used to create backend instances
		std::map<std::string, std::shared_ptr<KnowledgeGraphPlugin>> loadedPlugins_;
		// a counter used to generate unique IDs
		uint32_t backendIndex_;
        // an identifier for this manager
        uint32_t managerID_;

		std::shared_ptr<KnowledgeGraphPlugin> loadBackendPlugin(const std::string &path);

        /**
         * Remove a reasoner from this manager.
         * @reasoner a reasoner.
         */
        void removeBackend(const std::shared_ptr<DefinedKnowledgeGraph> &reasoner);
	};

	// a macro for static registration of a knowledge graph type.
	// knowledge graph types registered with this macro are builtin knowledge graphs that are not
	// loaded from a plugin.
	#define KNOWROB_BUILTIN_BACKEND(Name,Type) class Type ## _Registration{ static bool isRegistered; }; \
		bool Type ## _Registration::isRegistered = KnowledgeGraphManager::addFactory<Type>(Name);
}

#endif //KNOWROB_KG_MANAGER_H_
