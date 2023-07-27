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
#include "knowrob/queries/QueryEngine.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/backend/KnowledgeGraphManager.h"
#include "ThreadPool.h"

namespace knowrob {
    /**
     * The main interface to the knowledge base system implementing
     * its 'tell' and 'ask' interface.
     */
	class KnowledgeBase : public QueryEngine {
	public:
	    /**
	     * @param config a property tree used to configure this.
	     */
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

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
        ThreadPool& threadPool() { return *threadPool_; }

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const FormulaPtr &query, int queryFlags) override;

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const GraphQueryPtr &graphQuery) override;

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const LiteralPtr &query, int queryFlags) override;

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const LabeledLiteralPtr &query, int queryFlags) override;

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<KnowledgeGraphManager> backendManager_;
		std::shared_ptr<ThreadPool> threadPool_;

		void loadConfiguration(const boost::property_tree::ptree &config);
	};

    using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGE_BASE_H
