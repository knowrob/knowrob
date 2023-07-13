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
#include "knowrob/semweb/KnowledgeGraph.h"
#include "Statement.h"
#include "knowrob/queries/AnswerQueue.h"
#include "knowrob/queries/MultiModalPipeline.h"
#include "knowrob/queries/QueryEngine.h"

namespace knowrob {
	class KnowledgeBase : public QueryEngine {
	public:
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

        /**
         * Each KnowledgeBase uses a central knowledge graph that represents
         * different characteristics of agents, task they execute and environments
         * in which tasks are executed.
         * @return a knowledge graph.
         */
        const auto& knowledgeGraph() const { return knowledgeGraph_; }

        /**
         * Project a statement into the extensional database (EDB) where factual
         * knowledge is stored. However, each reasoner may use its own EDB backend.
         * Per default the given statement is projected into each known EDB where
         * the reasoner defines the predicate referred to in the query.
         * A particular EDB backend can be selected via the @reasonerID parameter.
         * @param statement A statement.
         * @param reasonerID The ID of a reasoner or '*' to select all.
         * @return true if the statement was inserted into at least one EDB.
         */
        bool projectIntoEDB(const Statement &statement, const std::string &reasonerID="*");

        /**
         * Projects a series of statements into extensional databases (EDBs) where factual
         * knowledge is stored.
         * @param statement A statement.
         * @param reasonerID The ID of a reasoner or '*' to select all.
         * @return true if the statement was inserted into at least one EDB.
         */
        bool projectIntoEDB(const std::list<Statement> &statements, const std::string &reasonerID="*");

        // Override QueryEngine
        BufferedAnswersPtr submitQuery(const FormulaPtr &query, int queryFlags) override;

        // Override QueryEngine
        BufferedAnswersPtr submitQuery(const std::vector<LiteralPtr> &literal,
                                       const ModalityLabelPtr &label,
                                       int queryFlags) override;

        // Override QueryEngine
        BufferedAnswersPtr submitQuery(const LiteralPtr &query, int queryFlags) override;

        // Override QueryEngine
        BufferedAnswersPtr submitQuery(const LabeledLiteralPtr &query, int queryFlags) override;

        /**
         * @param query
         * @param handler
         * @deprecated
         */
        void runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler);

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
        std::list<KnowledgeGraphPtr> knowledgeGraphs_;
        std::list<MultiModalPipeline> pipelines_;

		void loadConfiguration(const boost::property_tree::ptree &config);
	};

    using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGE_BASE_H
