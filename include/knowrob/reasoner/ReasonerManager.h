/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_MANAGER_H_
#define KNOWROB_REASONER_MANAGER_H_

#include "knowrob/reasoner/TypedReasonerFactory.h"
#include "knowrob/reasoner/ReasonerPlugin.h"
#include "knowrob/reasoner/DefinedPredicate.h"
#include "knowrob/reasoner/DefinedReasoner.h"

namespace knowrob {
	/**
	 * Manages a set of available reasoning subsystems.
	 */
	class ReasonerManager {
	public:
		ReasonerManager();
        ~ReasonerManager();

        /**
         * @param managerID the ID of a reasoner manager
         * @return the reasoner manager, or nullptr if ID is unknown
         */
        static ReasonerManager* getReasonerManager(uint32_t managerID);

		/**
		 * Add a reasoner factory to the manager.
		 * Note that factories for shared libraries are created on the fly, and thus
		 * do not need to be added manually.
		 * @param typeName the name of the reasoner type
		 * @param factory a reasoner factory
		 */
		static bool addReasonerFactory(const std::string &typeName, const std::shared_ptr<ReasonerFactory> &factory);

		template<class T> static bool addReasonerFactory(const std::string &typeName)
		{ return addReasonerFactory(typeName, std::make_shared<TypedReasonerFactory<T>>(typeName)); }

		/**
		 * Load a new reasoner instance into the reasoner manager.
		 * The type of the reasoner is determined based on either the value of
		 * "type" or "lib" in the property tree root. The tree is further used
		 * to generate a reasoner configuration used by the created reasoner.
		 * Reasoner factories for libraries are created on the fly, the ones
		 * for built-in reasoner types need to be added to the reasoner manager before.
		 * @param config a property tree holding a reasoner configuration
		 */
		void loadReasoner(const boost::property_tree::ptree &config);

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
		std::shared_ptr<DefinedReasoner> getReasonerWithID(const std::string &reasonerID);

        /**
         * Add a reasoner to this manager.
         * @reasoner a reasoner.
         */
        std::shared_ptr<DefinedReasoner> addReasoner(
                const std::string &reasonerID, const std::shared_ptr<IReasoner> &reasoner);

        /**
         * @return map of all reasoner defined by this manager.
         */
        const auto& reasonerPool() const  { return reasonerPool_; }

        auto managerID() const  { return managerID_; }

	private:
		// maps reasoner type name to factory used to create instances of that type
		static std::map<std::string, std::shared_ptr<ReasonerFactory>> reasonerFactories_;
        // maps manager id to manager
        static std::map<uint32_t, ReasonerManager*> reasonerManagers_;
        // counts number of initialized managers
        static uint32_t managerIDCounter_;
        // mutex used to interact with static variables
        std::mutex staticMutex_;
		// pool of all reasoner instances created via this manager
		// maps reasoner ID to reasoner instance.
		std::map<std::string, std::shared_ptr<DefinedReasoner>> reasonerPool_;
		// maps plugin names to factories used to create reasoner instances
		std::map<std::string, std::shared_ptr<ReasonerPlugin>> loadedPlugins_;
		// a counter used to generate unique IDs
		uint32_t reasonerIndex_;
        // an identifier for this manager
        uint32_t managerID_;

		std::shared_ptr<ReasonerPlugin> loadReasonerPlugin(const std::string &path);

        /**
         * Remove a reasoner from this manager.
         * @reasoner a reasoner.
         */
        void removeReasoner(const std::shared_ptr<DefinedReasoner> &reasoner);
	};

	// a macro for static registration of a reasoner type.
	// reasoner types registered with this macro are builtin reasoners that are not
	// loaded from a plugin.
	#define KNOWROB_BUILTIN_REASONER(Name,Type) class Type ## _Registration{ static bool isRegistered; }; \
		bool Type ## _Registration::isRegistered = ReasonerManager::addReasonerFactory<Type>(Name);
}

#endif //KNOWROB_REASONER_MANAGER_H_
