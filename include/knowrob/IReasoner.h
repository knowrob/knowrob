/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_IREASONER_H_
#define KNOWROB_IREASONER_H_

// STD
#include <memory>
// BOOST
#include <boost/property_tree/ptree.hpp>
// FMT
#include <fmt/core.h>
// KnowRob
#include "knowrob/terms/Term.h"
#include <knowrob/queries/QueryInstance.h>
#include <knowrob/data_sources.h>

namespace knowrob {
	/**
	 * A configuration of a reasoner.
	 * Each instance of a reasoner type may have its own configuration.
	 */
	class ReasonerConfiguration {
	public:
		/**
		 * Load a reasoner configuration from a property tree.
		 * @param ptree a property tree.
		 */
		void loadPropertyTree(const boost::property_tree::ptree &ptree);

		std::list<std::pair<TermPtr,TermPtr>> settings;
		std::list<std::shared_ptr<DataFile>> dataFiles;
		std::list<std::shared_ptr<FactBase>> factBases;
		std::list<std::shared_ptr<RuleBase>> ruleBases;
	private:
		void loadSettings(const TermPtr &key_t, const boost::property_tree::ptree &ptree);
	};

	using DataFileLoader = std::function<bool(const std::shared_ptr<DataFile>&)>;

	/**
	 * Flags indicating the capability of a reasoner.
	 * Flags can be combined into a bitmask of all reasoner capabilities.
	 */
	enum ReasonerCapability : unsigned long {
		CAPABILITY_NONE = 0x0,
		/** The reasoner can answer conjunctive queries */
		CAPABILITY_CONJUNCTIVE_QUERIES = 0x1,
		/** The reasoner can answer disjunctive queries */
		CAPABILITY_DISJUNCTIVE_QUERIES = 0x2
	};
	
	/**
	 * An interface for reasoning subsystems.
	 */
	class IReasoner {
	public:
		virtual ~IReasoner()= default;

		/**
		 * @param format
		 * @param fn
		 */
		void addDataFileHandler(const std::string &format, const DataFileLoader& fn);

		/**
		 *
		 * @param dataFile
		 * @return
		 */
		bool loadDataFile(const DataFilePtr &dataFile);

		/**
		 * Load a reasoner configuration.
		 * The knowledge base system only calls this function once for
		 * each reasoner instance.
		 */
		virtual bool loadConfiguration(const ReasonerConfiguration &cfg) = 0;

		/**
		 * Get the description of a predicate currently defined by this reasoner.
		 * A predicate is thought to be currently defined if it is defined by the reasoner,
		 * or imported in some way such that the reasoner can evaluate it.
		 *
		 * @param indicator a predicate indicator
		 * @return a predicate description if the predicate is a defined one or null otherwise.
		 */
		virtual std::shared_ptr<PredicateDescription> getPredicateDescription(
				const std::shared_ptr<PredicateIndicator> &indicator) = 0;

		/**
		 * @return bitmask of reasoner capabilities.
		 */
		virtual unsigned long getCapabilities() const = 0;

		/**
		 * @param capability a reasoner capability.
		 * @return true of this reasoner has the capability.
		 */
		bool hasCapability(ReasonerCapability capability) const
		{ return (getCapabilities() & capability); }

		/**
		 * Indicate that a new query needs to be evaluated.
		 * Note that different instances of @uninstantiatedQuery are to be evaluated.
		 * Answers computed over all instances of the query can be published via a
		 * shared message channel.
		 * @param queryID a query ID
		 * @param outputStream an output stream where answers can be published
		 * @param goal a query
		 */
		virtual void startQuery(uint32_t queryID, const std::shared_ptr<const Query> &uninstantiatedQuery) = 0;

		/**
		 * Adds a substitution to an active query request.
		 * The substitution is used to create an instance of the input query.
		 * Note that this function is eventually called rapidly many times in case many
		 * different substitutions are generated e.g. through other sub-queries.
		 * @param queryID a query ID
		 * @param substitution a substitution
		 */
		virtual void runQueryInstance(uint32_t queryID, const QueryInstancePtr &queryInstance) = 0;
		
		/**
		 * Indicate that a query has been completed, i.e. that no further
		 * substitutions will be added to the query request.
		 * Note that query evaluation may still be in progress when this function is called.
		 * A flag is used to indicate to the reasoner whether it should stop evaluation immediately,
		 * or whether it should complete the evaluation before finishing the query request.
		 * Note that this flag is just a request, and a reasoner may not be able to immediately stop.
		 * In such a case it is also ok if the reasoner stops as soon as possible.
		 * The call should further be non-blocking, i.e. it should not wait for the reasoner being
		 * done with the query.
		 * @param queryID a query ID
		 * @param isImmediateStopRequested a flag indicating whether a reasoner may stop query evaluation immediately
		 */
		virtual void finishQuery(uint32_t queryID,
								 const std::shared_ptr<QueryResultStream::Channel> &outputStream,
								 bool isImmediateStopRequested) = 0;

	protected:
		std::map<std::string, DataFileLoader> dataFileHandler_;

		virtual bool loadDataFileWithUnknownFormat(const DataFilePtr&) { return false; }

		friend class ReasonerManager;
	};

	/**
	 * A reasoner-related runtime error.
	 */
	class ReasonerError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit ReasonerError(const char *fmt, Args&& ... args)
		: std::runtime_error(fmt::format(fmt, args...)) {}
	};
}

/**
 * Define a reasoner plugin.
 * The macro generates two functions that are used as entry points for
 * loading the plugin.
 * First, a factory function is defined that creates instances of @classType.
 * This will only work when @classType has a single argument constructor that
 * accepts a string as argument (the reasoner instance ID).
 * Second, a function is generated that exposes the plugin name.
 * @classType the type of the reasoner, must be a subclass of IReasoner
 * @pluginName a plugin identifier, e.g. the name of the reasoner type.
 */
#define REASONER_PLUGIN(classType, pluginName) extern "C" { \
		std::shared_ptr<knowrob::IReasoner> knowrob_createReasoner(const std::string &reasonerID) \
			{ return std::make_shared<classType>(reasonerID); } \
		const char* knowrob_getPluginName() { return pluginName; } }

#endif //KNOWROB_IREASONER_H_
