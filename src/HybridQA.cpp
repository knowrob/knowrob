/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/HybridQA.h>
#include <knowrob/Blackboard.h>

using namespace knowrob;

HybridQA::HybridQA()
{
	// create a PrologReasoner
	knowrob::ReasonerConfiguration reasonerConfig;
	reasonerConfig.dataFiles.push_back(
			std::make_shared<knowrob::PrologDataFile>("tests/prolog/kb1.pl"));
	prologReasoner_ = std::make_shared<knowrob::PrologReasoner>();
	//auto reasoner = std::make_shared<knowrob::MongologReasoner>();
	prologReasoner_->initialize(reasonerConfig);

	reasonerManager_ = std::make_shared<ReasonerManager>();
	reasonerManager_->addReasoner(prologReasoner_);
}

std::shared_ptr<Query> HybridQA::parseQuery(const std::string &queryString)
{
	auto term =prologReasoner_->readTerm(queryString);
	return PrologQuery::toQuery(term);
}

void HybridQA::runQuery(const std::shared_ptr<Query> &query, QueryResultHandler &handler) {
	auto bbq = std::make_shared<knowrob::QueryResultQueue>();
	auto bb = std::make_shared<Blackboard>(reasonerManager_, bbq, query);
	QueryResultPtr solution;

	bb->start();
	do {
		solution = bbq->pop_front();
		if(QueryResultStream::isEOS(solution)) {
			break;
		}
	} while(handler.pushQueryResult(solution));
}
