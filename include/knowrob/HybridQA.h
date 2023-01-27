/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_HYBRIDQA_H
#define KNOWROB_HYBRIDQA_H

#include <memory>
// BOOST
#include <boost/property_tree/ptree.hpp>
// KnowRob
#include <knowrob/queries.h>
#include <knowrob/reasoner.h>
#include <knowrob/prolog/PrologReasoner.h>
#include "Blackboard.h"

namespace knowrob {
	class QueryResultHandler {
	public:
		virtual bool pushQueryResult(const QueryResultPtr &solution) = 0;
	};

	class HybridQA {
	public:
		explicit HybridQA(const boost::property_tree::ptree &config);

		std::shared_ptr<const Query> parseQuery(const std::string &queryString);

        int callPrologDirect(const std::string &queryString);

		void runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler);

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<PrologReasoner> prologReasoner_;

        std::shared_ptr<knowrob::QueryResultQueue> bbq_;
        std::shared_ptr<Blackboard> bb_;

		void loadConfiguration(const boost::property_tree::ptree &config);
	};
};

#endif //KNOWROB_HYBRIDQA_H
