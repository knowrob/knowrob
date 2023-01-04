/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/HybridQA.h>
#include <knowrob/Blackboard.h>

using namespace knowrob;

HybridQA::HybridQA(const boost::property_tree::ptree &config)
{
	reasonerManager_ = std::make_shared<ReasonerManager>();
	loadConfiguration(config);
	// create a PrologReasoner used only for parsing queries
	prologReasoner_ = std::make_shared<knowrob::PrologReasoner>("prolog_user");
	prologReasoner_->initialize(ReasonerConfiguration());
}

void HybridQA::loadConfiguration(const boost::property_tree::ptree &config)
{
	for(const auto &pair : config.get_child("reasoner")) {
		try {
			reasonerManager_->loadReasoner(pair.second);
		}
		catch(std::exception& e) {
			KB_ERROR("failed to load a reasoner: {}", e.what());
		}
	}
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
