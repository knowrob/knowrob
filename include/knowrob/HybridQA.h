/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_HYBRIDQA_H
#define KNOWROB_HYBRIDQA_H

// STD
#include <memory>
// KnowRob
#include <knowrob/queries.h>
#include <knowrob/ReasonerManager.h>
#include <knowrob/prolog/PrologReasoner.h>

namespace knowrob {
	class QueryResultHandler {
	public:
		virtual bool pushQueryResult(const QueryResultPtr &solution) = 0;
	};

	class HybridQA {
	public:
		HybridQA();

		std::shared_ptr<Query> parseQuery(const std::string &queryString);

		void runQuery(const std::shared_ptr<Query> &query, QueryResultHandler &handler);

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<PrologReasoner> prologReasoner_;
	};
};

#endif //KNOWROB_HYBRIDQA_H
