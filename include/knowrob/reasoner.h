/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_H_
#define KNOWROB_REASONER_H_

// STD
#include <list>
#include <map>
#include <memory>
// BOOST
#include <boost/property_tree/ptree.hpp>
#include <utility>
// KnowRob
#include <knowrob/ThreadPool.h>
#include "knowrob/terms/Term.h"
#include <knowrob/IReasoner.h>

namespace knowrob {
	/**
	 * Abstract reasoner factory.
	 * Provides an interface for the creation of IReasoner instances.
	 */
	class ReasonerFactory {
	public:
		virtual ~ReasonerFactory()= default;

		/**
		 * Create a new reasoner instance.
		 * @param reasonerID the ID of the reasoner in the knowledge base.
		 * @return the reasoner created.
		 */
		virtual std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) = 0;

		/**
		 * @return name of the reasoner type for which the factory can create instances.
		 */
		virtual const std::string& name() const = 0;
	};

	/**
	 * A reasoner factory implementation for native reasoner types.
	 * @tparam T the type of reasoner.
	 */
	template<class T> class TypedReasonerFactory : public ReasonerFactory {
	public:
		/**
		 * @param name name of the reasoner type for which the factory can create instances.
		 */
		explicit TypedReasonerFactory(std::string name) : name_(std::move(name)) {}

		// Override ReasonerFactory
		std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) override
		{ return std::make_shared<T>(reasonerID); }

		// Override ReasonerFactory
		const std::string& name() const override {  return name_; }
	protected:
		std::string name_;
	};

	/**
	 * A reasoner factory that uses a reasoner shared library
	 * for creation of reasoner instances.
	 * Reasoner plugins are usually defined in shared libraries
	 * through the REASONER_PLUGIN macro.
	 * They need to expose a set of functions that are used as
	 * an entry point for loading the plugin.
	 */
	class ReasonerPlugin : public ReasonerFactory {
	public:
		/**
		 * @param dllPath the name or path of the shared library.
		 */
		explicit ReasonerPlugin(std::string dllPath);

		~ReasonerPlugin() override;

		/**
		 * Cannot be copy-assigned.
		 */
		ReasonerPlugin(const ReasonerPlugin&) = delete;

		/**
		 * @return true if the shared library was loaded successfully.
		 */
		bool isLoaded();

		/**
		 * Try loading the shared library from filesystem.
		 * Note that, on unix-based systems, the LD_LIBRARY_PATH environment
		 * variable is used to locate the library.
		 * @return true on success.
		 */
		bool loadDLL();

		// Override ReasonerFactory
		std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) override;

		// Override ReasonerFactory
		const std::string& name() const override {  return name_; };

	protected:
		const std::string dllPath_;
		std::string name_;
		// handle of opened library
		void *handle_;
		// a factory function used to create new instances of a reasoner.
		std::shared_ptr<IReasoner> (*create_)(const std::string &reasonerID);
		// a function that returns the name of the plugin
		char* (*get_name_)();
	};

	/**
	 * A reasoner with a name managed by the reasoner manager.
	 */
	class ManagedReasoner {
	public:
		/**
		 * @param name the name of the reasoner, unique within manager
		 * @param reasoner the reasoner instance
		 */
		ManagedReasoner(std::string name, const std::shared_ptr<IReasoner> &reasoner)
		: name_(std::move(name)), reasoner_(reasoner) {}

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<IReasoner>& operator()() const { return reasoner_; }

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<IReasoner>& reasoner() const { return reasoner_; }

		/**
		 * @return the reasoner name.
		 */
		const std::string& name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<IReasoner> reasoner_;
	};

	/**
	 * A predicate description with associated reasoners that can
	 * evaluate the predicate.
	 */
	class PredicateDefinition {
	public:
		/**
		 * @param indicator the indicator of the described predicate.
		 */
		explicit PredicateDefinition(const std::shared_ptr<PredicateIndicator> &indicator);

		/**
		 * Add a reasoner to this description.
		 * @param managedReasoner a managed reasoner.
		 * @param definition a predicate description.
		 * @return true if the reasoner was added successfully.
		 */
		bool addReasoner(const std::shared_ptr<ManagedReasoner> &managedReasoner,
						 const std::shared_ptr<PredicateDescription> &definition);

		/**
		 * @return the type of the described predicate.
		 */
		PredicateType predicateType() const { return predicateType_; }

		/**
		 * @return set of reasoners associated to this description.
		 */
		const std::set<std::shared_ptr<ManagedReasoner>>& reasonerEnsemble() const { return reasonerEnsemble_; }

	protected:
		const std::shared_ptr<PredicateIndicator> indicator_;
		PredicateType predicateType_;
		std::set<std::shared_ptr<ManagedReasoner>> reasonerEnsemble_;
	};

	/**
	 * Manages a set of available reasoning subsystems.
	 */
	class ReasonerManager {
	public:
		ReasonerManager();

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
		 * Add a reasoner to this manager.
		 * @reasoner a reasoner.
		 */
		std::shared_ptr<ManagedReasoner> addReasoner(
				const std::string &reasonerID, const std::shared_ptr<IReasoner> &reasoner);

		/**
		 * Remove a reasoner from this manager.
		 * @reasoner a reasoner.
		 */
		void removeReasoner(const std::shared_ptr<ManagedReasoner> &reasoner);

		/**
		 * Get the definition of a predicate.
		 * @param predicate the predicate in question
		 * @return a predicate definition
		 */
		std::shared_ptr<PredicateDefinition> getPredicateDefinition(
				const std::shared_ptr<PredicateIndicator> &predicate);

		/**
		 * @param reasonerID a reasoner ID string.
		 * @return a reasoner instance or a null pointer reference.
		 */
		std::shared_ptr<ManagedReasoner> getReasonerWithID(const std::string &reasonerID);

	private:
		// maps reasoner type name to factory used to create instances of that type
		static std::map<std::string, std::shared_ptr<ReasonerFactory>> reasonerFactories_;
		// pool of all reasoner instances created via this manager
		// maps reasoner ID to reasoner instance.
		std::map<std::string, std::shared_ptr<ManagedReasoner>> reasonerPool_;
		// maps plugin names to factories used to create reasoner instances
		std::map<std::string, std::shared_ptr<ReasonerPlugin>> loadedPlugins_;
		// a counter used to generate unique IDs
		uint32_t reasonerIndex_;

		std::shared_ptr<ReasonerPlugin> loadReasonerPlugin(const std::string &path);
	};

	// a macro for static registration of a reasoner type.
	// reasoner types registered with this macro are builtin reasoners that are not
	// loaded from a plugin.
	#define KNOWROB_BUILTIN_REASONER(Name,Type) class Type ## _Registration{ static bool isRegistered; }; \
		bool Type ## _Registration::isRegistered = ReasonerManager::addReasonerFactory<Type>(Name);
}

#endif //KNOWROB_REASONER_H_
