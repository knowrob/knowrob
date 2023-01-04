/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONER_MANAGER_H__
#define __KNOWROB_REASONER_MANAGER_H__

// STD
#include <list>
#include <map>
#include <memory>
// BOOST
#include <boost/property_tree/ptree.hpp>
// KnowRob
#include <knowrob/ThreadPool.h>
#include <knowrob/terms.h>
#include <knowrob/IReasoner.h>

namespace knowrob {

	class ReasonerFactory {
	public:
		virtual ~ReasonerFactory()= default;

		virtual std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) = 0;

		virtual const std::string& name() const = 0;
	};

	template<class T> class TypedReasonerFactory : public ReasonerFactory {
	public:
		TypedReasonerFactory(const std::string &name) : name_(name) {}

		std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) override
		{ return std::make_shared<T>(reasonerID); }

		const std::string& name() const override {  return name_; };
	protected:
		std::string name_;
	};

	class ReasonerPlugin : public ReasonerFactory {
	public:
		ReasonerPlugin(const std::string &dllPath);

		ReasonerPlugin(const ReasonerPlugin&) = delete;

		~ReasonerPlugin();

		bool isLoaded();

		bool loadDLL();

		std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID);

		const std::string& name() const override {  return name_; };

	protected:
		const std::string dllPath_;
		std::string name_;
		void *handle_;
		// a factory function used to create new instances of a reasoner.
		std::shared_ptr<IReasoner> (*create_)(const std::string &reasonerID);
		char* (*get_name_)();
	};

	/**
	 * Manages a set of available reasoning subsystems.
	 */
	class ReasonerManager {
	public:
		ReasonerManager();

		void loadReasoner(const boost::property_tree::ptree &config);

		void addReasonerFactory(const std::string &typeName, const std::shared_ptr<ReasonerFactory> &factory);

		/** Add a reasoner to this manager.
		 * @reasoner a reasoner.
		 */
		void addReasoner(const std::shared_ptr<IReasoner> &reasoner);

		/** Remove a reasoner from this manager.
		 * @reasoner a reasoner.
		 */
		void removeReasoner(const std::shared_ptr<IReasoner> &reasoner);

		/** Get list of reasoner that can handle given predicate.
		 *
		 * @param predicate the predicate in question
		 * @return an essemble of reasoner that can handle the predicate
		 */
		std::list<std::shared_ptr<IReasoner>> getReasonerForPredicate(const PredicateIndicator &predicate);

	private:
		std::list<std::shared_ptr<IReasoner>> reasonerPool_;
		std::map<std::string, std::shared_ptr<ReasonerFactory>> reasonerFactories_;
		std::map<std::string, std::shared_ptr<ReasonerPlugin>> loadedPlugins_;
		uint32_t reasonerIndex_;

		std::shared_ptr<ReasonerPlugin> loadReasonerPlugin(const std::string &path);
	};
}

#endif //__KNOWROB_REASONER_MANAGER_H__
