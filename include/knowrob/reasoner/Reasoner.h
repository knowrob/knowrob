/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_H_
#define KNOWROB_REASONER_H_

#include <memory>
#include <filesystem>
#include <fmt/core.h>

#include "knowrob/terms/Term.h"
#include "knowrob/reasoner/ReasonerConfig.h"
#include "knowrob/DataSource.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/queries/GraphQuery.h"
#include "knowrob/backend/KnowledgeGraph.h"
#include "knowrob/DataSourceHandler.h"

namespace knowrob {
	/**
	 * Flags indicating the truth mode of a reasoner distinguishing between
	 * closed world and open world semantics.
	 */
	enum TruthMode : uint8_t {
		OPEN_WORLD = 1,
		CLOSED_WORLD
	};

	/**
	 * A reasoner is a component that can infer new knowledge.
	 * It does so by evaluating axioms or rules that are defined by the reasoner.
	 * The axioms and rules may refer to extensional data which is stored in the
	 * knowledge base in form of a knowledge graph, and which may need to be mirrored
	 * into the reasoner's own data backend in case it cannot operate directly
	 * on the central extensional database.
	 * Note that a reasoner is also a data source handler, i.e. data which is needed
	 * by the reasoner to operate which is not stored in a backend.
	 */
	class Reasoner : public DataSourceHandler {
	public:
        Reasoner();
		virtual ~Reasoner()= default;

        /**
         * @return ID of the manager that created the reasoner.
         */
        uint32_t managerID() const { return reasonerManagerID_; }

        /**
         * Set the data backend of this reasoner.
		 */
        virtual void setDataBackend(const DataBackendPtr &backend) = 0;

		/**
		 * Load a reasoner configuration.
		 * The knowledge base system only calls this function once for each reasoner instance.
		 * @param config a ReasonerConfig object.
		 */
		virtual bool loadConfig(const ReasonerConfig &config) = 0;

		/**
		 * The truth mode of this reasoner determines whether it operates
		 * with open world or closed world semantics.
		 * @return the truth mode of this reasoner.
		 */
		virtual TruthMode getTruthMode() const = 0;

		/**
		 * Get the description of a predicate currently defined by this reasoner.
		 * A predicate is thought to be currently defined if the reasoner can submitQuery it.
		 *
		 * @param indicator a predicate indicator
		 * @return a predicate description if the predicate is a defined one or null otherwise.
		 */
		virtual PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) = 0;

		/**
		 * Get the description of the predicate which is associated with a literal.
		 * A null reference will be returned in case that the property term of the literal is a variable.
		 * @param literal a literal.
		 * @return a predicate description or a null reference.
		 */
		PredicateDescriptionPtr getLiteralDescription(const RDFLiteral &literal);

		/**
		 * Start the reasoner.
		 * This is in particular intended to start any bottom-up evaluation processes
		 * that run in the background.
		 */
		virtual void start() = 0;

        /**
         * Stop the reasoner, and destroy all resources.
         * No further queries can be submitted after this function has been called.
         */
		virtual void stop() = 0;

		/**
		 * Submit a query to the reasoner.
		 * The query is represented by a literal and a context.
		 * The evaluation of the query is performed asynchronously, the result of this function
		 * is a buffer that can be used to retrieve the results of the query at a later point in time.
		 * @param literal a literal representing the query.
		 * @param ctx a query context.
		 * @return a buffer that can be used to retrieve the results of the query.
		 */
        virtual AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx) = 0;

	protected:
        uint32_t reasonerManagerID_;

        void setReasonerManager(uint32_t managerID);

		friend class ReasonerManager;
	};

	/**
	 * In some cases reasoner and backend cannot really be separated.
	 * The whole point of the separation is to re-use backends with different reasoners.
	 * So if this is not possible, then the reasoner can also implement the backend interface.
	 */
	class ReasonerWithBackend : public Reasoner, public DataBackend {
	public:
		ReasonerWithBackend() : Reasoner(), DataBackend() {}

		// avoid a self-reference
		void setDataBackend(const DataBackendPtr &backend) final {}
	};
}

#endif //KNOWROB_REASONER_H_
