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
#include "knowrob/reasoner/ReasonerConfiguration.h"
#include "knowrob/DataSource.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/queries/GraphQuery.h"
#include "knowrob/semweb/KnowledgeGraph.h"

namespace knowrob {
	/**
	 * Flags indicating the capability of a reasoner.
	 * Flags can be combined into a bitmask of all reasoner capabilities.
	 */
	enum ReasonerCapability : unsigned long {
		CAPABILITY_NONE = 1 << 0,
		CAPABILITY_NEGATIONS = 1 << 1,
		/** The reasoner can answer conjunctive queries */
		CAPABILITY_CONJUNCTIVE_QUERIES = 1 << 1,
		/** The reasoner can answer disjunctive queries */
		CAPABILITY_DISJUNCTIVE_QUERIES = 1 << 2,
        CAPABILITY_TOP_DOWN_EVALUATION  = 1 << 3,
        CAPABILITY_BOTTOM_UP_EVALUATION = 1 << 4
	};
	
	/**
	 * An interface for reasoning subsystems.
	 */
	class Reasoner {
	public:
        Reasoner();
		virtual ~Reasoner()= default;

        /**
         * @return ID of the manager that created the reasoner.
         */
        uint32_t reasonerManagerID() const { return reasonerManagerID_; }

        virtual void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) = 0;

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
		 * A predicate is thought to be currently defined if the reasoner can submitQuery it.
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

		bool canEvaluate(const RDFLiteral &literal);

        virtual AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx) = 0;

	protected:
		std::map<std::string, DataSourceLoader> dataSourceHandler_;
        uint32_t reasonerManagerID_;

		virtual bool loadDataSourceWithUnknownFormat(const DataSourcePtr&) { return false; }

        void setReasonerManager(uint32_t managerID);

		friend class ReasonerManager;
	};
}

#endif //KNOWROB_IREASONER_H_
