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
#include "knowrob/queries/AllocatedQuery.h"
#include "knowrob/reasoner/ReasonerConfiguration.h"
#include "knowrob/DataSource.h"
#include "knowrob/Statement.h"
#include "knowrob/semweb/GraphPattern.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/semweb/GraphQuery.h"

namespace knowrob {
	/**
	 * Flags indicating the capability of a reasoner.
	 * Flags can be combined into a bitmask of all reasoner capabilities.
	 */
	enum ReasonerCapability : unsigned long {
		CAPABILITY_NONE = 1 << 0,
		/** The reasoner can answer conjunctive queries */
		CAPABILITY_CONJUNCTIVE_QUERIES = 1 << 1,
		/** The reasoner can answer disjunctive queries */
		CAPABILITY_DISJUNCTIVE_QUERIES = 1 << 2,
        CAPABILITY_TOP_DOWN_EVALUATION  = 1 << 3,
        CAPABILITY_BOTTOM_UP_EVALUATION = 1 << 4
        /** The reasoner can store/recover data to/from a local path. */
        //CAPABILITY_IMPORT_EXPORT = 0x4,
        /** The reasoner can insert new facts into the knowledge base at runtime. */
        //CAPABILITY_DYNAMIC_ASSERTIONS = 0x8
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

        virtual bool runQuery(const AllocatedQueryPtr &query) = 0;

        /**
         * Project a statement into the extensional database (EDB) where factual
         * knowledge is stored.
         * @param statement A statement.
         * @return true if the statement was inserted into the EDB used by the reasoner.
         */
        //virtual bool projectIntoEDB(const Statement &statement) { return false; }

        /**
         * Export data to local filesystem.
         * This function is only assumed to be implemented if CAPABILITY_IMPORT_EXPORT is set.
         * @param path an existing local filesystem path.
         * @return true on success.
         */
        //virtual bool exportData(const std::filesystem::path &path) { return false; }

        /**
         * Import data from local filesystem.
         * This function is only assumed to be implemented if CAPABILITY_IMPORT_EXPORT is set.
         * @param path an existing local filesystem path.
         * @return true on success.
         */
        //virtual bool importData(const std::filesystem::path &path) { return false; }

	protected:
		std::map<std::string, DataSourceLoader> dataSourceHandler_;
        uint32_t reasonerManagerID_;

		virtual bool loadDataSourceWithUnknownFormat(const DataSourcePtr&) { return false; }

        void setReasonerManager(uint32_t managerID);

		friend class ReasonerManager;
	};
}

#endif //KNOWROB_IREASONER_H_
