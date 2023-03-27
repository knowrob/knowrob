/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWLEDGEBASE_H
#define KNOWROB_KNOWLEDGEBASE_H

#include <memory>
// BOOST
#include <boost/property_tree/ptree.hpp>
// KnowRob
#include "knowrob/reasoner/ReasonerManager.h"
#include "Statement.h"
#include <knowrob/reasoner/prolog/PrologReasoner.h>

namespace knowrob {
	class KnowledgeBase {
	public:
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

		std::shared_ptr<const Query> parseQuery(const std::string &queryString);

		void runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler);

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

        int callPrologDirect(const std::string &queryString);

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<PrologReasoner> prologReasoner_;

		void loadConfiguration(const boost::property_tree::ptree &config);
	};

    using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGEBASE_H
