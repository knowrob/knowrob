/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWLEDGE_BASE_H
#define KNOWROB_KNOWLEDGE_BASE_H

#include <memory>
#include <boost/property_tree/ptree.hpp>
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/backend/BackendManager.h"
#include "ThreadPool.h"
#include "knowrob/queries/DependencyGraph.h"
#include "knowrob/queries/QueryPipeline.h"
#include "knowrob/queries/QueryContext.h"

namespace knowrob {
    enum QueryFlag {
        QUERY_FLAG_ALL_SOLUTIONS     = 1 << 0,
        QUERY_FLAG_ONE_SOLUTION      = 1 << 1,
        QUERY_FLAG_PERSIST_SOLUTIONS = 1 << 2,
        QUERY_FLAG_UNIQUE_SOLUTIONS  = 1 << 3
    };

    class RDFComputable : public RDFLiteral
    {
    public:
        RDFComputable(const RDFLiteral &lit, const std::vector<std::shared_ptr<Reasoner>> &reasonerList)
        : RDFLiteral(lit), reasonerList_(reasonerList) {}

        const auto& reasonerList() const { return reasonerList_; }
    protected:
        std::vector<std::shared_ptr<Reasoner>> reasonerList_;
    };
    using RDFComputablePtr = std::shared_ptr<RDFComputable>;

    /**
     * The main interface to the knowledge base system implementing
     * its 'tell' and 'ask' interface.
     */
	class KnowledgeBase {
	public:
	    /**
	     * @param config a property tree used to configure this.
	     */
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

	    /**
	     * @param configFile path to file that encodes a boost property tree used to configure the KB.
	     */
		explicit KnowledgeBase(const std::string_view &configFile);

        /**
         * Asserts a proposition into the knowledge base.
         * @param tripleData data representing the proposition.
         * @return true on success.
         */
        bool insert(const StatementData &proposition);

        /**
         * Asserts a sequence of propositions into the knowledge base.
         * @param tripleData data representing a list of proposition.
         * @return true on success.
         */
        bool insert(const std::vector<StatementData> &propositions);

        /**
         * @return a thread pool owned by this.
         */
        auto& threadPool() { return *threadPool_; }

		/**
		 * @return the central knowledge graph
		 */
        auto centralKG() const { return centralKG_; }

        /**
         * @return the vocabulary of this knowledge base, i.e. all known properties and classes
         */
        auto vocabulary() const { return centralKG()->vocabulary(); }

        /**
         * @return import hierarchy of named graphs
         */
        auto importHierarchy() const { return centralKG()->importHierarchy(); }

        /**
         * Evaluate a query represented as a vector of literals.
         * The call is non-blocking and returns a stream of answers.
         * @param literals a vector of literals
         * @param label an optional modalFrame label
         * @return a stream of query results
         */
        AnswerBufferPtr submitQuery(const GraphQueryPtr &graphQuery);

        /**
         * Evaluate a query represented as a Literal.
         * The call is non-blocking and returns a stream of answers.
         * @param query a literal
         * @return a stream of query results
         */
        AnswerBufferPtr submitQuery(const LiteralPtr &query, const QueryContextPtr &ctx);

        /**
         * Evaluate a query represented as a Formula.
         * The call is non-blocking and returns a stream of answers.
         * @param query a formula
         * @return a stream of query results
         */
        AnswerBufferPtr submitQuery(const FormulaPtr &query, const QueryContextPtr &ctx);

		auto& reasonerManager() const { return reasonerManager_; }

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<BackendManager> backendManager_;
		std::shared_ptr<ThreadPool> threadPool_;
		std::shared_ptr<KnowledgeGraph> centralKG_;

		void loadConfiguration(const boost::property_tree::ptree &config);

        static std::vector<RDFComputablePtr> createComputationSequence(
                const std::list<DependencyNodePtr> &dependencyGroup);

        void createComputationPipeline(
            const std::shared_ptr<QueryPipeline> &pipeline,
            const std::vector<RDFComputablePtr> &computableLiterals,
            const std::shared_ptr<AnswerBroadcaster> &pipelineInput,
            const std::shared_ptr<AnswerBroadcaster> &pipelineOutput,
            const QueryContextPtr &ctx);
	};

    using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGE_BASE_H
