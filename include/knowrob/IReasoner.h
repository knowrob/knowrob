/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_IREASONER_H__
#define __KNOWROB_IREASONER_H__

// STD
#include <memory>
// BOOST
#include <boost/property_tree/ptree.hpp>
// KnowRob
#include <knowrob/terms.h>
#include <knowrob/queries.h>
#include <knowrob/data_sources.h>

namespace knowrob {
	class ReasonerConfiguration {
	public:
		ReasonerConfiguration(const boost::property_tree::ptree &ptree);
		ReasonerConfiguration() = default;

		std::list<std::shared_ptr<DataFile>> dataFiles;
		std::list<std::shared_ptr<FactBase>> factBases;
		std::list<std::shared_ptr<RuleBase>> ruleBases;
	};

	class ReasonerError : public std::runtime_error {
	public:
		ReasonerError(const std::string& what = "") : std::runtime_error(what) {}
	};
	
	/**
	 * The interface for reasoning subsystems.
	 */
	class IReasoner {
	public:
		virtual ~IReasoner()= default;

		/** Initialize this reasoner.
		 * This will only be called once when the reasoner is loaded.
		 */
		virtual bool initialize(const ReasonerConfiguration &cfg) = 0;

		/** Find out whether a given predicate is defined by this reasoner.
		 *
		 * @param indicator a predicate indicator
		 * @return true if the indicated predicate is currently defined.
		 */
		virtual bool isCurrentPredicate(const PredicateIndicator &indicator) = 0;

		/**
		 */
		virtual void startQuery(uint32_t queryID,
			const std::shared_ptr<QueryResultStream::Channel> &outputStream,
			const std::shared_ptr<Query> &goal) = 0;

		/**
		 */
		virtual void pushSubstitution(uint32_t queryID,
			const SubstitutionPtr &substitution) = 0;
		
		/**
		 */
		virtual void finishQuery(uint32_t queryID,
			bool isImmediateStopRequested) = 0;
	};
}

#define REASONER_PLUGIN(classType, pluginName) extern "C" { \
		std::shared_ptr<IReasoner> knowrob_createReasoner(const std::string &reasonerID) \
			{ return std::make_shared<classType>(reasonerID); } \
		const char* knowrob_getPluginName() { return pluginName; } }

#endif //__KNOWROB_IREASONER_H__
