/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_IREASONER_H_
#define KNOWROB_IREASONER_H_

#include <memory>
#include <filesystem>
#include <fmt/core.h>

#include "knowrob/terms/Term.h"
#include "knowrob/queries/QueryInstance.h"
#include "knowrob/reasoner/ReasonerConfiguration.h"
#include "knowrob/DataSource.h"
#include "knowrob/Statement.h"
#include "knowrob/semweb/GraphPattern.h"

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
		CAPABILITY_DISJUNCTIVE_QUERIES = 0x2,
        /** The reasoner can store/recover data to/from a local path. */
        CAPABILITY_IMPORT_EXPORT = 0x4,
        /** The reasoner can insert new facts into the knowledge base at runtime. */
        CAPABILITY_DYNAMIC_ASSERTIONS = 0x8
	};
	
	/**
	 * An interface for reasoning subsystems.
	 */
	class IReasoner {
	public:
        IReasoner();
		virtual ~IReasoner()= default;

        /**
         * @return ID of the manager that created the reasoner.
         */
        uint32_t reasonerManagerID() const { return reasonerManagerID_; }

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

        /*
        virtual void startGraphCompletion(uint32_t requestID,
                                          const std::shared_ptr<PossibleWorld> &possibleWorld,
                                          const std::shared_ptr<GraphPattern> &focus) = 0;

        virtual void stopGraphCompletion(uint32_t requestID, bool isImmediateStopRequested) = 0;
         */


		/**
		 * Indicate that a new query needs to be evaluated.
		 * Note that different instances of @uninstantiatedQuery are to be evaluated.
		 * Answers computed over all instances of the query can be published via a
		 * shared message channel.
		 * @param queryID a query ID
		 * @param outputStream an output stream where answers can be published
		 * @param goal a query
		 * @deprecated
		 */
		virtual void startQuery(uint32_t queryID, const std::shared_ptr<const Query> &uninstantiatedQuery) = 0;

		/**
		 * Adds a substitution to an active query request.
		 * The substitution is used to create an instance of the input query.
		 * Note that this function is eventually called rapidly many times in case many
		 * different substitutions are generated e.g. through other sub-queries.
		 * @param queryID a query ID
		 * @param substitution a substitution
		 * @deprecated
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
		 * @deprecated
		 */
		virtual void finishQuery(uint32_t queryID,
								 const std::shared_ptr<AnswerStream::Channel> &outputStream,
								 bool isImmediateStopRequested) = 0;

        /**
         * Project a statement into the extensional database (EDB) where factual
         * knowledge is stored.
         * @param statement A statement.
         * @return true if the statement was inserted into the EDB used by the reasoner.
         */
        virtual bool projectIntoEDB(const Statement &statement) { return false; }

        /**
         * Export data to local filesystem.
         * This function is only assumed to be implemented if CAPABILITY_IMPORT_EXPORT is set.
         * @param path an existing local filesystem path.
         * @return true on success.
         */
        virtual bool exportData(const std::filesystem::path &path) { return false; }

        /**
         * Import data from local filesystem.
         * This function is only assumed to be implemented if CAPABILITY_IMPORT_EXPORT is set.
         * @param path an existing local filesystem path.
         * @return true on success.
         */
        virtual bool importData(const std::filesystem::path &path) { return false; }

	protected:
		std::map<std::string, DataSourceLoader> dataSourceHandler_;
        uint32_t reasonerManagerID_;

		virtual bool loadDataSourceWithUnknownFormat(const DataSourcePtr&) { return false; }

        void setReasonerManager(uint32_t managerID);

		friend class ReasonerManager;
	};
}

#endif //KNOWROB_IREASONER_H_
