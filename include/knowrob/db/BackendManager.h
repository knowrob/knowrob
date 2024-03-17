/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BACKEND_MANAGER_H_
#define KNOWROB_BACKEND_MANAGER_H_

#include <map>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include "knowrob/semweb/Vocabulary.h"
#include "TypedBackendFactory.h"
#include "BackendPlugin.h"
#include "DefinedBackend.h"
#include "QueryableBackend.h"

namespace knowrob {
	/**
	 * Manages a set of available backend subsystems.
	 */
	class BackendManager {
	public:
		/**
		 * @param kb the knowledge base this manager is associated with.
		 */
		explicit BackendManager(const std::shared_ptr<Vocabulary> &vocabulary);

		~BackendManager();

		/**
		 * @param managerID the ID of a backend manager
		 * @return the backend manager, or nullptr if ID is unknown
		 */
		static BackendManager *getManager(uint32_t managerID);

		/**
		 * Add a backend factory to the manager.
		 * Note that factories for shared libraries are created on the fly, and thus
		 * do not need to be added manually.
		 * @param typeName the name of the backend type
		 * @param factory a backend factory
		 * @return true if the factory was added, false otherwise
		 */
		static bool addFactory(const std::string &typeName, const std::shared_ptr<BackendFactory> &factory);

		/**
		 * Add a typed backend factory to the manager.
		 * @tparam T a backend type.
		 * @param typeName the name of the backend type
		 * @return true if the factory was added, false otherwise
		 */
		template<class T>
		static bool addFactory(const std::string &typeName) {
			return addFactory(typeName, std::make_shared<TypedBackendFactory<T>>(typeName));
		}

		/**
		 * Load a new backend instance into the backend manager.
		 * The type of the backend is determined based on either the value of
		 * "type" or "lib" in the property tree root. The tree is further used
		 * to generate a backend configuration used by the created backend.
		 * Backend factories for libraries are created on the fly, the ones
		 * for built-in backend types need to be added to the backend manager before.
		 * @param config a property tree holding a backend configuration
		 */
		DataBackendPtr loadBackend(const boost::property_tree::ptree &config);

		/**
		 * @param backendID a backend ID string.
		 * @return a backend instance or a null pointer reference.
		 */
		std::shared_ptr<DefinedBackend> getBackendWithID(std::string_view backendID);

		/**
		 * Add a backend to this manager.
		 * @reasoner a defined backend.
		 */
		std::shared_ptr<DefinedBackend> addBackend(
				const std::string &reasonerID, const DataBackendPtr &backend);

		/**
		 * Add a backend to this manager.
		 * @param backend a defined backend.
		 */
		void addBackend(const std::shared_ptr<DefinedBackend> &backend);

		/**
		 * @return map of all KG defined by this manager.
		 */
		const auto &backendPool() const { return backendPool_; }

		/**
		 * @return map of all persistent backends defined by this manager.
		 */
		const auto &persistent() const { return persistent_; }

		/**
		 * @return map of all queryable backends defined by this manager.
		 */
		const auto &queryable() const { return queryable_; }

		/**
		 * @return the manager ID.
		 */
		auto managerID() const { return managerID_; }

		/**
		 * @return the vocabulary associated with this manager.
		 */
		auto &vocabulary() const { return vocabulary_; }

	private:
		std::shared_ptr<Vocabulary> vocabulary_;
		// maps backend id to manager
		static std::map<uint32_t, BackendManager *> backendManagers_;
		// counts number of initialized managers
		static uint32_t managerIDCounter_;
		// mutex used to interact with static variables
		std::mutex staticMutex_;
		// pool of all backend instances created via this manager
		// maps backend ID to backend instance.
		std::map<std::string, std::shared_ptr<DefinedBackend>, std::less<>> backendPool_;
		std::map<std::string, QueryableBackendPtr> persistent_;
		std::map<std::string, QueryableBackendPtr> queryable_;
		// maps plugin names to factories used to create backend instances
		std::map<std::string, std::shared_ptr<BackendPlugin>> loadedPlugins_;
		// a counter used to generate unique IDs
		uint32_t backendIndex_;
		// an identifier for this manager
		uint32_t managerID_;

		std::shared_ptr<BackendPlugin> loadBackendPlugin(const std::string &path);

		/**
		 * Remove a reasoner from this manager.
		 * @reasoner a reasoner.
		 */
		void removeBackend(const std::shared_ptr<DefinedBackend> &reasoner);

		void initBackend(const std::shared_ptr<DefinedBackend> &definedKG);
	};
}

	// a macro for static registration of a knowledge graph type.
	// knowledge graph types registered with this macro are builtin knowledge graphs that are not
	// loaded from a plugin.
#define KNOWROB_BUILTIN_BACKEND(Name, Type) class Type ## _Registration{ static bool isRegistered; }; \
        bool Type ## _Registration::isRegistered = knowrob::BackendManager::addFactory<Type>(Name);

#endif //KNOWROB_BACKEND_MANAGER_H_
