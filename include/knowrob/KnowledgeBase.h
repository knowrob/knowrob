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
#include "knowrob/backend/KnowledgeGraph.h"
#include "Statement.h"
#include "knowrob/queries/AnswerQueue.h"
#include "knowrob/queries/QueryPipeline.h"
#include "knowrob/queries/QueryEngine.h"
#include "ThreadPool.h"
#include "knowrob/backend/KnowledgeGraphManager.h"

namespace knowrob {
	class KnowledgeBase : public QueryEngine {
	public:
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

        ThreadPool& threadPool() { return *threadPool_; }

        bool insert(const TripleData &tripleData);

        bool insert(const std::vector<TripleData> &tripleData);

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const FormulaPtr &query, int queryFlags) override;

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const GraphQueryPtr &graphQuery) override;

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const LiteralPtr &query, int queryFlags) override;

        // Override QueryEngine
        AnswerBufferPtr submitQuery(const LabeledLiteralPtr &query, int queryFlags) override;

        /**
         * @param query
         * @param handler
         * @deprecated
         */
        void runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler);

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<KnowledgeGraphManager> backendManager_;
		std::shared_ptr<ThreadPool> threadPool_;

		void loadConfiguration(const boost::property_tree::ptree &config);
	};

    using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGE_BASE_H
