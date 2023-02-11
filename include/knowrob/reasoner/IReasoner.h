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
// FMT
#include <fmt/core.h>
// KnowRob
#include "knowrob/terms/Term.h"
#include "knowrob/queries/QueryInstance.h"
#include "knowrob/reasoner/ReasonerConfiguration.h"
#include "knowrob/DataSource.h"

namespace knowrob {
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
		void addDataSourceHandler(const std::string &format,
                                  const std::function<bool(const DataSourcePtr &)> &fn);

		/**
		 *
		 * @param dataSource
		 * @return
		 */
		bool loadDataSource(const DataSourcePtr &dataSource);

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
		bool hasCapability(ReasonerCapability capability) const;

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
		std::map<std::string, DataSourceLoader> dataSourceHandler_;

		virtual bool loadDataSourceWithUnknownFormat(const DataSourcePtr&) { return false; }

		friend class ReasonerManager;
	};
}

#endif //KNOWROB_IREASONER_H_
