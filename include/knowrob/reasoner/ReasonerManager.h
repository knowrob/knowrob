/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_MANAGER_H_
#define KNOWROB_REASONER_MANAGER_H_

#include "knowrob/reasoner/TypedReasonerFactory.h"
#include "knowrob/reasoner/ReasonerPlugin.h"
#include "knowrob/reasoner/DefinedPredicate.h"
#include "knowrob/reasoner/DefinedReasoner.h"
#include "knowrob/backend/BackendManager.h"
#include "knowrob/KnowledgeBase.h"
#include "ReasonerModule.h"

namespace knowrob {
	/**
	 * Manages a set of available reasoning subsystems.
	 */
	class ReasonerManager {
	public:
		/**
		 * Create a new reasoner manager.
		 * @param kb the knowledge base associated with this manager.
		 * @param backendManager the backend manager associated with this manager.
		 */
		ReasonerManager(KnowledgeBase *kb, const std::shared_ptr<BackendManager> &backendManager);

		~ReasonerManager();

		/**
		 * @param managerID the ID of a reasoner manager
		 * @return the reasoner manager, or nullptr if ID is unknown
		 */
		static ReasonerManager *getReasonerManager(uint32_t managerID);

		/**
		 * Add a reasoner factory to the manager.
		 * Note that factories for shared libraries are created on the fly, and thus
		 * do not need to be added manually.
		 * @param typeName the name of the reasoner type
		 * @param factory a reasoner factory
		 */
		static bool addReasonerFactory(std::string_view typeName, const std::shared_ptr<ReasonerFactory> &factory);

		/**
		 * Add a typed reasoner factory to the manager.
		 * @param typeName the name of the reasoner type
		 * @param factory a reasoner factory
		 */
		template<class T>
		static bool addReasonerFactory(std::string_view typeName) {
			return addReasonerFactory(typeName, std::make_shared<TypedReasonerFactory<T>>(typeName));
		}

		/**
		 * Load a new reasoner instance into the reasoner manager.
		 * The type of the reasoner is determined based on either the value of
		 * "type" or "lib" in the property tree root. The tree is further used
		 * to generate a reasoner configuration used by the created reasoner.
		 * Reasoner factories for libraries are created on the fly, the ones
		 * for built-in reasoner types need to be added to the reasoner manager before.
		 * @param config a property tree holding a reasoner configuration
		 */
		std::shared_ptr<DefinedReasoner> loadReasoner(const boost::property_tree::ptree &config);

		/**
		 * Get the definition of a predicate.
		 * @param predicate the predicate in question
		 * @return a predicate definition
		 */
		std::shared_ptr<DefinedPredicate> getPredicateDefinition(
				const std::shared_ptr<PredicateIndicator> &predicate);

		/**
		 * @param reasonerID a reasoner ID string.
		 * @return a reasoner instance or a null pointer reference.
		 */
		std::shared_ptr<DefinedReasoner> getReasonerWithID(std::string_view reasonerID);

		/**
		 * Return the backend associated with a reasoner if any.
		 * @param reasoner a defined reasoner.
		 * @return a backend or a null reference.
		 */
		std::shared_ptr<DataBackend> getReasonerBackend(const std::shared_ptr<DefinedReasoner> &reasoner);

		/**
		 * Add a reasoner to this manager.
		 * @reasoner a reasoner.
		 */
		std::shared_ptr<DefinedReasoner> addReasoner(std::string_view reasonerID, const std::shared_ptr<Reasoner> &reasoner);

		/**
		 * @return map of all reasoner defined by this manager.
		 */
		const auto &reasonerPool() const { return reasonerPool_; }

		/**
		 * @return the ID of this manager.
		 */
		auto managerID() const { return managerID_; }

		/**
		 * @return the knowledge base associated with this manager.
		 */
		auto kb() const { return kb_; }

	private:
		KnowledgeBase *kb_;
		std::shared_ptr<BackendManager> backendManager_;
		// maps reasoner type name to factory used to create instances of that type
		static std::map<std::string, std::shared_ptr<ReasonerFactory>, std::less<>> reasonerFactories_;
		// maps manager id to manager
		static std::map<uint32_t, ReasonerManager *> reasonerManagers_;
		// counts number of initialized managers
		static uint32_t managerIDCounter_;
		// mutex used to interact with static variables
		std::mutex staticMutex_;
		// pool of all reasoner instances created via this manager
		// maps reasoner ID to reasoner instance.
		std::map<std::string, std::shared_ptr<DefinedReasoner>, std::less<>> reasonerPool_;
		// maps plugin names to factories used to create reasoner instances
		std::map<std::string, std::shared_ptr<ReasonerPlugin>, std::less<>> loadedPlugins_;
		std::map<std::string, std::shared_ptr<ReasonerModule>, std::less<>> loadedModules_;
		// maps reasoner to their backends
		std::map<std::string_view, std::shared_ptr<DataBackend>, std::less<>> reasonerBackends_;
		// a counter used to generate unique IDs
		uint32_t reasonerIndex_;
		// an identifier for this manager
		uint32_t managerID_;

		std::shared_ptr<ReasonerPlugin> loadReasonerPlugin(std::string_view path);

		std::shared_ptr<ReasonerModule> loadReasonerModule(std::string_view path, std::string_view type);

		void setDataBackend(const std::shared_ptr<Reasoner> &reasoner, const std::shared_ptr<DataBackend> &dataBackend);

		/**
		 * Remove a reasoner from this manager.
		 * @reasoner a reasoner.
		 */
		void removeReasoner(const std::shared_ptr<DefinedReasoner> &reasoner);
	};
}

// a macro for static registration of a reasoner type.
// reasoner types registered with this macro are builtin reasoners that are not
// loaded from a plugin.
#define KNOWROB_BUILTIN_REASONER(Name, Type) class Type ## _Registration{ static bool isRegistered; }; \
        bool Type ## _Registration::isRegistered = ReasonerManager::addReasonerFactory<Type>(Name);

#endif //KNOWROB_REASONER_MANAGER_H_
